#include "lbr_acquisition/readback.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <string_view>
#include <type_traits>

namespace lbr_acquisition {
namespace {
constexpr std::size_t kFileLimit = 16 * 1024 * 1024;
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
std::uint32_t crc32(std::string_view bytes) {
  std::uint32_t crc = 0xffffffffU;
  for (const unsigned char byte : bytes) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320U : 0U);
  }
  return crc ^ 0xffffffffU;
}
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
  const std::vector<OfflineEvent> &expected;
  const std::string &descriptor;
  std::map<std::uint16_t, std::string> schemas;
  std::map<std::uint16_t, std::uint16_t> channels;
  std::set<std::string> schema_names;
  std::set<std::string> topics;
  std::size_t messages = 0;

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
      const std::map<std::string, std::string> wanted{
          {"clock", "host_monotonic"},
          {"units", "rad"},
          {"joint_order", "A1,A2,A3,A4,A5,A6,A7"},
          {"source", "synthetic"}};
      require(values == wanted, "channel metadata mismatch");
      p.end();
      channels.emplace(id, schema_id);
    } else if (record.opcode == 5 && in_chunk) {
      require(messages < expected.size() && messages < 10000,
              "unexpected/excess message");
      const auto channel = p.number<std::uint16_t>();
      require(channels.count(channel), "undeclared channel");
      require(p.number<std::uint32_t>() == 0, "sequence must be unspecified");
      const auto log = p.number<std::uint64_t>();
      const auto publish = p.number<std::uint64_t>();
      require(p.left() <= 4096, "protobuf payload limit exceeded");
      std::visit(
          [&](const auto &expected_message) {
            require(log == expected_message.host_monotonic_ns() &&
                        publish == log,
                    "container timestamp mismatch");
            require(schemas.at(channels.at(channel)) ==
                        expected_message.GetTypeName(),
                    "message type/order mismatch");
            const auto payload = p.take(p.left());
            require(expected_message.IsInitialized(),
                    "uninitialized expected message");
            require(payload == expected_message.SerializeAsString(),
                    "message bytes differ");
          },
          expected[messages]);
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
    while (inner.left()) {
      auto record = next(inner);
      data_record(record, true);
    }
    if (messages != before) {
      std::uint64_t minimum = UINT64_MAX, maximum = 0;
      for (auto i = before; i < messages; ++i)
        std::visit(
            [&](const auto &message) {
              minimum = std::min(minimum, message.host_monotonic_ns());
              maximum = std::max(maximum, message.host_monotonic_ns());
            },
            expected[i]);
      require(start == minimum && finish == maximum,
              "chunk time range mismatch");
    }
  }
};
} // namespace

void verify_container(const std::filesystem::path &path,
                      const std::vector<OfflineEvent> &expected,
                      const std::string &descriptor) {
  require(expected.size() <= 10000 && descriptor.size() <= 65536,
          "argument limits exceeded");
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  require(static_cast<bool>(file), "cannot open file");
  const auto length = file.tellg();
  require(length >= 0 && length <= static_cast<std::streamoff>(kFileLimit),
          "file limit exceeded");
  file.seekg(0);
  std::string bytes(static_cast<std::size_t>(length), '\0');
  require(static_cast<bool>(file.read(bytes.data(), bytes.size())),
          "file read failed");
  require(file.peek() == std::char_traits<char>::eof() && !file.bad(),
          "file changed/read failed");
  Cursor outer{bytes};
  require(outer.take(8) == kMagic, "initial magic mismatch");
  auto header = next(outer);
  require(header.opcode == 1 && header.payload.text().empty(),
          "initial header/profile mismatch");
  require(!header.payload.text().empty(), "missing library identification");
  header.payload.end();
  Scan scan{expected, descriptor, {}, {}, {}, {}, 0};
  bool ended = false;
  while (outer.left()) {
    const auto record_start = outer.pos;
    auto record = next(outer);
    if (record.opcode == 15) {
      require(!ended, "duplicate data end");
      require(record.payload.number<std::uint32_t>() ==
                  crc32(std::string_view(bytes).substr(0, record_start)),
              "data checksum mismatch");
      record.payload.end();
      ended = true;
      break;
    }
    if (record.opcode == 6)
      scan.chunk(record.payload);
    else
      scan.data_record(record, false);
  }
  require(ended, "missing data end");
  auto footer = next(outer);
  require(footer.opcode == 2, "missing final footer");
  require(footer.payload.number<std::uint64_t>() == 0 &&
              footer.payload.number<std::uint64_t>() == 0 &&
              footer.payload.number<std::uint32_t>() == 0,
          "summary is not permitted");
  footer.payload.end();
  require(outer.take(8) == kMagic, "trailing magic mismatch");
  outer.end();
  require(scan.schemas.size() == 2 && scan.channels.size() == 2,
          "exactly two definitions required");
  require(scan.messages == expected.size(), "missing messages");
}
} // namespace lbr_acquisition
