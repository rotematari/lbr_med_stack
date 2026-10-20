#include "lbr_acquisition/readback.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <string_view>
#include <type_traits>
#include <cstring>
#include <openssl/sha.h>

namespace lbr_acquisition {
void CaptureSummary::observe(bool state, uint64_t host_ns, std::string_view payload) {
  if (payload.size() > 4096 || events == UINT64_MAX)
    throw InvalidInput("summary payload/count limit");
  std::array<unsigned char, 32 + 1 + 8 + 8 + 4096> input{};
  std::copy(digest.begin(), digest.end(), input.begin());
  input[32] = state ? 0 : 1;
  for (unsigned i = 0; i < 8; ++i) {
    input[33 + i] = static_cast<unsigned char>(host_ns >> (8 * i));
    input[41 + i] = static_cast<unsigned char>(payload.size() >> (8 * i));
  }
  std::memcpy(input.data() + 49, payload.data(), payload.size());
  if (!SHA256(input.data(), 49 + payload.size(), digest.data()))
    throw IoError("summary digest failed");
  ++events;
  if (state) {
    if (!states) first_state_ns = host_ns;
    ++states;
    last_state_ns = host_ns;
  }
}
void CaptureSummary::observe(const OfflineEvent &event) {
  std::visit([&](const auto &m) {
    if (!m.IsInitialized()) throw InvalidInput("uninitialized summary event");
    observe(event.index() == 0, m.host_monotonic_ns(), m.SerializeAsString());
  }, event);
}
bool CaptureSummary::operator==(const CaptureSummary &other) const noexcept {
  return events == other.events && states == other.states &&
         first_state_ns == other.first_state_ns && last_state_ns == other.last_state_ns &&
         digest == other.digest;
}
namespace {
constexpr std::size_t kRecordLimit = 1024 * 1024;
constexpr std::string_view kMagic("\x89MCAP0\r\n", 8);
[[noreturn]] void fail(const std::string &reason) {
  throw IoError("readback: " + reason);
}
void require(bool ok, const std::string &reason) {
  if (!ok)
    fail(reason);
}

// A cursor is always bounded by its enclosing record, including nested records.
struct Cursor {
  std::string_view data;
  std::size_t pos = 0;
  std::size_t left() const { return data.size() - pos; }
  std::string_view take(std::uint64_t n) {
    require(n <= left(), "truncated field");
    const auto result = data.substr(pos, static_cast<std::size_t>(n));
    pos += static_cast<std::size_t>(n);
    return result;
  }
  template <class T> T number() {
    static_assert(std::is_unsigned_v<T>);
    const auto raw = take(sizeof(T));
    T value = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
      value |= static_cast<T>(static_cast<unsigned char>(raw[i])) << (8 * i);
    return value;
  }
  std::string_view text() { return take(number<std::uint32_t>()); }
  void end() const { require(left() == 0, "unexpected record suffix"); }
};
void update_crc32(uint32_t &crc, std::string_view bytes) {
  static const auto table = [] {
    std::array<uint32_t, 256> values{};
    for (uint32_t i = 0; i < 256; ++i) {
      auto value = i;
      for (int bit = 0; bit < 8; ++bit)
        value = (value >> 1) ^ ((value & 1) ? 0xedb88320U : 0U);
      values[i] = value;
    }
    return values;
  }();
  for (const unsigned char byte : bytes)
    crc = (crc >> 8) ^ table[(crc ^ byte) & 255];
}
uint32_t crc32(std::string_view bytes) {
  uint32_t crc = 0xffffffffU;
  update_crc32(crc, bytes);
  return crc ^ 0xffffffffU;
}
// Reads bounded records, never the complete file. CRC covers the exact bytes
// before DataEnd, including magic and the initial Header.
struct FileInput {
  std::ifstream file;
  uint64_t size = 0, position = 0;
  uint32_t crc = 0xffffffffU;
  FileInput(const std::filesystem::path &path, uint64_t limit)
      : file(path, std::ios::binary | std::ios::ate) {
    require(static_cast<bool>(file), "cannot open file");
    const auto n = file.tellg();
    require(n >= 0 && static_cast<uint64_t>(n) <= limit, "file limit exceeded");
    size = static_cast<uint64_t>(n);
    file.seekg(0);
  }
  std::string take(uint64_t n) {
    require(n <= kRecordLimit && n <= size - position, "truncated/oversize file read");
    std::string result(static_cast<size_t>(n), '\0');
    if (n) require(static_cast<bool>(file.read(result.data(), result.size())), "file read failed");
    position += n;
    return result;
  }
  void end() {
    require(position == size && file.peek() == std::char_traits<char>::eof() && !file.bad(),
            "trailing bytes/file changed/read failed");
  }
};

struct Record {
  std::uint8_t opcode;
  Cursor payload;
};
Record next(Cursor &source) {
  const auto opcode = source.number<std::uint8_t>();
  const auto length = source.number<std::uint64_t>();
  require(length <= kRecordLimit, "record limit exceeded");
  return {opcode, {source.take(length)}};
}
struct Scan {
  const std::string &descriptor;
  CaptureSource expected_source;
  std::optional<CaptureSource> observed_source;
  std::map<std::uint16_t, std::string> schemas;
  std::map<std::uint16_t, std::uint16_t> channels;
  std::set<std::string> schema_names;
  std::set<std::string> topics;
  uint64_t messages = 0;
  CaptureSummary observed;
  uint64_t max_events, chunk_min = UINT64_MAX, chunk_max = 0;

  void data_record(Record &record, bool in_chunk) {
    auto &p = record.payload;
    if (record.opcode == 3) {
      const auto id = p.number<std::uint16_t>();
      const auto name = p.text();
      require(id && schemas.size() < 2 && !schemas.count(id),
              "invalid schema id/count");
      require(name == "lbr.acquisition.v1.State" ||
                  name == "lbr.acquisition.v1.Submission",
              "unsupported schema name");
      require(schema_names.insert(std::string(name)).second,
              "duplicate schema type");
      require(p.text() == "protobuf", "schema encoding mismatch");
      const auto size = p.number<std::uint32_t>();
      require(size <= 65536, "descriptor limit exceeded");
      require(p.take(size) == descriptor, "descriptor mismatch");
      p.end();
      schemas.emplace(id, name);
    } else if (record.opcode == 4) {
      const auto id = p.number<std::uint16_t>();
      const auto schema_id = p.number<std::uint16_t>();
      require(channels.size() < 2 && !channels.count(id) &&
                  schemas.count(schema_id),
              "invalid channel id/count/reference");
      const auto topic = p.text();
      const bool state = schemas.at(schema_id) == "lbr.acquisition.v1.State";
      require(topic == (state ? "/lbr_acquisition/state"
                              : "/lbr_acquisition/submission"),
              "topic/type mismatch");
      require(topics.insert(std::string(topic)).second, "duplicate topic");
      require(p.text() == "protobuf", "message encoding mismatch");
      Cursor metadata{p.take(p.number<std::uint32_t>())};
      std::map<std::string, std::string> values;
      while (metadata.left()) {
        const std::string key(metadata.text());
        const std::string value(metadata.text());
        require(values.emplace(key, value).second, "duplicate metadata key");
      }
      const auto source_entry = values.find("source");
      require(source_entry != values.end(), "missing channel source");
      CaptureSource channel_source;
      if (source_entry->second == "synthetic")
        channel_source = CaptureSource::Synthetic;
      else if (source_entry->second == "fri-sdk")
        channel_source = CaptureSource::FriSdk;
      else
        fail("unsupported channel source");
      require(channel_source == expected_source &&
                  (!observed_source || *observed_source == channel_source),
              "channel source mismatch");
      const std::map<std::string, std::string> wanted{
          {"clock", "host_monotonic"},
          {"position_units", "rad"},
          {"torque_units", "Nm"},
          {"joint_order", "A1,A2,A3,A4,A5,A6,A7"},
          {"source", capture_source_name(channel_source)}};
      require(values == wanted, "channel metadata mismatch");
      observed_source = channel_source;
      p.end();
      channels.emplace(id, schema_id);
    } else if (record.opcode == 5 && in_chunk) {
      require(messages < max_events,
              "unexpected/excess message");
      const auto channel = p.number<std::uint16_t>();
      require(channels.count(channel), "undeclared channel");
      require(p.number<std::uint32_t>() == 0, "sequence must be unspecified");
      const auto log = p.number<std::uint64_t>();
      const auto publish = p.number<std::uint64_t>();
      require(p.left() <= 4096, "protobuf payload limit exceeded");
      require(publish == log, "container timestamp mismatch");
      const bool state = schemas.at(channels.at(channel)) == "lbr.acquisition.v1.State";
      observed.observe(state, log, p.take(p.left()));
      chunk_min = std::min(chunk_min, log);
      chunk_max = std::max(chunk_max, log);
      ++messages;
    } else {
      fail("record not allowed in data section");
    }
  }
  void chunk(Cursor &p) {
    const auto start = p.number<std::uint64_t>();
    const auto finish = p.number<std::uint64_t>();
    const auto uncompressed = p.number<std::uint64_t>();
    require(uncompressed <= kRecordLimit, "chunk limit exceeded");
    const auto checksum = p.number<std::uint32_t>();
    require(p.text().empty(), "compressed chunk rejected");
    const auto stored = p.number<std::uint64_t>();
    require(stored == uncompressed, "uncompressed size mismatch");
    const auto records = p.take(stored);
    p.end();
    require(crc32(records) == checksum, "chunk checksum mismatch");
    Cursor inner{records};
    const auto before = messages;
    chunk_min = UINT64_MAX; chunk_max = 0;
    while (inner.left()) {
      auto record = next(inner);
      data_record(record, true);
    }
    if (messages != before) {
      require(start == chunk_min && finish == chunk_max,
              "chunk time range mismatch");
    }
  }
};
} // namespace

CaptureSource verify_capture_container(const std::filesystem::path &path,
                                      const CaptureSummary &expected,
                                      const std::string &descriptor,
                                      CaptureSource source, CaptureLimits limits) {
  limits = checked_limits(limits);
  capture_source_name(source);
  require(expected.events <= limits.events && descriptor.size() <= 65536,
          "argument limits exceeded");
  FileInput file(path, limits.file_bytes);
  const auto magic = file.take(8);
  require(magic == kMagic, "initial magic mismatch");
  update_crc32(file.crc, magic);
  auto next_file = [&](bool checksum) {
    const auto header = file.take(9);
    Cursor cursor{header};
    const auto op = cursor.number<uint8_t>();
    const auto length = cursor.number<uint64_t>();
    auto payload = file.take(length);
    if (checksum && op != 15) {
      update_crc32(file.crc, header);
      update_crc32(file.crc, payload);
    }
    return std::make_pair(op, std::move(payload));
  };
  {
    const auto header = next_file(true);
    Cursor p{header.second};
    require(header.first == 1 && p.text().empty(), "initial header/profile mismatch");
    require(!p.text().empty(), "missing library identification");
    p.end();
  }
  Scan scan{descriptor, source, std::nullopt, {}, {}, {}, {}, 0, {}, limits.events};
  bool ended = false;
  while (file.position < file.size) {
    const auto stored = next_file(true);
    Record record{stored.first, {stored.second}};
    if (record.opcode == 15) {
      require(record.payload.number<uint32_t>() == (file.crc ^ 0xffffffffU),
              "data checksum mismatch");
      record.payload.end();
      ended = true;
      break;
    }
    if (record.opcode == 6) scan.chunk(record.payload);
    else scan.data_record(record, false);
  }
  require(ended, "missing data end");
  const auto stored_footer = next_file(false);
  Record footer{stored_footer.first, {stored_footer.second}};
  require(footer.opcode == 2, "missing final footer");
  require(footer.payload.number<uint64_t>() == 0 &&
              footer.payload.number<uint64_t>() == 0 &&
              footer.payload.number<uint32_t>() == 0, "summary is not permitted");
  footer.payload.end();
  require(file.take(8) == kMagic, "trailing magic mismatch");
  file.end();
  require(scan.schemas.size() == 2 && scan.channels.size() == 2,
          "exactly two definitions required");
  require(scan.observed == expected, "record count/clocks/digest differ from acknowledged events");
  require(scan.observed_source.has_value(), "missing channel source");
  return *scan.observed_source;
}

CaptureSource verify_container(const std::filesystem::path &path,
                               const std::vector<OfflineEvent> &expected,
                               const std::string &descriptor, CaptureSource source) {
  require(expected.size() <= CaptureLimits{}.events, "argument limits exceeded");
  CaptureSummary summary;
  for (const auto &event : expected) summary.observe(event);
  return verify_capture_container(path, summary, descriptor, source, {});
}
} // namespace lbr_acquisition
