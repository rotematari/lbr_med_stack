#include "lbr_acquisition/recorder.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

namespace {

thread_local bool track_allocations = false;
thread_local uint64_t tracked_allocations = 0;

[[noreturn]] void fail(const char *expression, int line) noexcept {
  std::fprintf(stderr, "CHECK failed at line %d: %s\n", line, expression);
  std::abort();
}

#define CHECK(expression)                                                      \
  do {                                                                         \
    if (!(expression))                                                         \
      fail(#expression, __LINE__);                                             \
  } while (false)

void *allocate(std::size_t size) {
  if (track_allocations)
    ++tracked_allocations;
  if (void *pointer = std::malloc(size == 0 ? 1 : size))
    return pointer;
  throw std::bad_alloc{};
}

void *allocate_aligned(std::size_t size, std::size_t alignment) {
  if (track_allocations)
    ++tracked_allocations;
  void *pointer = nullptr;
  if (posix_memalign(&pointer, alignment, size == 0 ? 1 : size) == 0)
    return pointer;
  throw std::bad_alloc{};
}

constexpr auto wait_limit = std::chrono::seconds(5);

class HookController {
public:
  void arm(unsigned point) {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(!enabled_);
    point_ = point;
    reached_ = false;
    released_ = false;
    enabled_ = true;
  }

  void arrive(unsigned point) noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || point != point_)
      return;
    reached_ = true;
    condition_.notify_all();
    if (!condition_.wait_for(lock, wait_limit, [this] { return released_; }))
      fail("hook release before timeout", __LINE__);
  }

  void wait_until_reached() {
    std::unique_lock<std::mutex> lock(mutex_);
    CHECK(condition_.wait_for(lock, wait_limit, [this] { return reached_; }));
  }

  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(enabled_ && reached_);
    released_ = true;
    enabled_ = false;
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned point_ = 0;
  bool enabled_ = false;
  bool reached_ = false;
  bool released_ = false;
};

HookController hooks;

} // namespace

void *operator new(std::size_t size) { return allocate(size); }
void *operator new[](std::size_t size) { return allocate(size); }
void operator delete(void *pointer) noexcept { std::free(pointer); }
void operator delete[](void *pointer) noexcept { std::free(pointer); }
void operator delete(void *pointer, std::size_t) noexcept {
  std::free(pointer);
}
void operator delete[](void *pointer, std::size_t) noexcept {
  std::free(pointer);
}
void *operator new(std::size_t size, std::align_val_t alignment) {
  return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void *operator new[](std::size_t size, std::align_val_t alignment) {
  return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void operator delete(void *pointer, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete[](void *pointer, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete(void *pointer, std::size_t, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete[](void *pointer, std::size_t, std::align_val_t) noexcept {
  std::free(pointer);
}

namespace lbr_acquisition::capture {
void test_hook(unsigned point) noexcept { hooks.arrive(point); }
} // namespace lbr_acquisition::capture

namespace {

using namespace lbr_acquisition::capture;

constexpr uint64_t run_id = 7;

Config config(std::size_t capacity = 8, uint64_t max_events = 10000,
              uint64_t duration_ns = 1'000'000,
              uint64_t first_sample_timeout_ns = 1'000'000) {
  return {run_id, duration_ns, first_sample_timeout_ns, max_events, capacity};
}

Event state(uint64_t id, uint64_t time_ns) {
  Event event{};
  event.kind = Kind::State;
  event.run_id = run_id;
  event.id = id;
  event.state_id = id;
  event.host_monotonic_ns = time_ns;
  for (std::size_t joint = 0; joint < event.position.size(); ++joint)
    event.position[joint] = static_cast<double>(id * 16 + joint) + 0.125;
  return event;
}

Event submission(uint64_t id, uint64_t state_id, uint64_t time_ns) {
  Event event = state(id, time_ns);
  event.kind = Kind::Submission;
  event.state_id = state_id;
  return event;
}

constexpr uint32_t failure_flag(Reason reason) {
  return uint32_t{1} << static_cast<uint32_t>(reason);
}

struct CollectingWriter : Writer {
  std::vector<Event> events;
  unsigned finishes = 0;

  void write(const Event &event) override { events.push_back(event); }
  void finish() override { ++finishes; }
};

void drain(Recorder &recorder, Writer &writer) {
  for (unsigned step = 0; step < 10002 && !recorder.status().writer_done;
       ++step)
    recorder.writer_step(writer);
  CHECK(recorder.status().writer_done);
}

class BlockingWriter final : public CollectingWriter {
public:
  enum class Point { Write, Finish };

  explicit BlockingWriter(Point point) : point_(point) {}

  void write(const Event &event) override {
    CollectingWriter::write(event);
    if (point_ == Point::Write)
      block();
  }

  void finish() override {
    ++finishes;
    if (point_ == Point::Finish)
      block();
  }

  void wait_until_entered() {
    std::unique_lock<std::mutex> lock(mutex_);
    CHECK(condition_.wait_for(lock, wait_limit, [this] { return entered_; }));
  }

  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(entered_);
    released_ = true;
    condition_.notify_all();
  }

private:
  void block() {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    CHECK(condition_.wait_for(lock, wait_limit, [this] { return released_; }));
  }

  Point point_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
  bool released_ = false;
};

void run_writer(Recorder &recorder, Writer &writer) {
  while (!recorder.status().writer_done) {
    if (!recorder.writer_step(writer))
      std::this_thread::yield();
  }
}

void blocked_writer_does_not_block_stop_or_status() {
  Recorder recorder(config());
  BlockingWriter writer(BlockingWriter::Point::Write);
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);

  std::thread consumer([&] { run_writer(recorder, writer); });
  writer.wait_until_entered();

  std::mutex stopped_mutex;
  std::condition_variable stopped_condition;
  bool stopped = false;
  bool stop_result = false;
  std::thread stopper([&] {
    stop_result = recorder.stop(run_id);
    {
      std::lock_guard<std::mutex> lock(stopped_mutex);
      stopped = true;
    }
    stopped_condition.notify_all();
  });
  {
    std::unique_lock<std::mutex> lock(stopped_mutex);
    CHECK(
        stopped_condition.wait_for(lock, wait_limit, [&] { return stopped; }));
  }
  stopper.join();
  CHECK(stop_result);

  const Status blocked = recorder.status();
  CHECK(blocked.phase == Phase::Finalizing);
  CHECK(blocked.reason == Reason::Manual);
  CHECK(blocked.writer_in_call);
  CHECK(!blocked.writer_done);
  CHECK(blocked.enqueued == 1 && blocked.written == 0);

  writer.release();
  consumer.join();
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Complete);
  CHECK(finished.attempted == 1 && finished.enqueued == 1);
  CHECK(finished.rejected == 0 && finished.written == 1);
  CHECK(finished.abandoned == 0 && finished.write_uncertain == 0);
  CHECK(writer.events.size() == 1 && writer.finishes == 1);
}

void blocked_writer_does_not_block_duration_expiry() {
  Recorder recorder(config(8, 10000, 20, 50));
  BlockingWriter writer(BlockingWriter::Point::Write);
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);

  std::thread consumer([&] { run_writer(recorder, writer); });
  writer.wait_until_entered();
  recorder.tick(121);
  const Status expired = recorder.status();
  CHECK(expired.phase == Phase::Finalizing);
  CHECK(expired.reason == Reason::Duration);
  CHECK(expired.writer_in_call && !expired.writer_done);
  CHECK(expired.enqueued == 1 && expired.written == 0);

  writer.release();
  consumer.join();
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Complete);
  CHECK(finished.reason == Reason::Duration);
  CHECK(finished.written == 1 && writer.finishes == 1);
}

void blocked_finish_has_truthful_live_status() {
  Recorder recorder(config());
  BlockingWriter writer(BlockingWriter::Point::Finish);
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);
  CHECK(recorder.stop(run_id));

  std::thread consumer([&] { run_writer(recorder, writer); });
  writer.wait_until_entered();
  const Status blocked = recorder.status();
  CHECK(blocked.phase == Phase::Finalizing);
  CHECK(blocked.reason == Reason::Manual);
  CHECK(blocked.writer_in_call && !blocked.writer_done);
  CHECK(blocked.written == 1);
  CHECK(recorder.stop(run_id));

  writer.release();
  consumer.join();
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Complete && finished.writer_done);
  CHECK(finished.written == 1 && writer.finishes == 1);
}

void capture_offer_does_not_allocate() {
  Recorder recorder(config());
  CollectingWriter writer;
  writer.events.reserve(2);
  CHECK(recorder.start(run_id, 100));

  Event measured = state(1, 101);
  Event commanded = submission(1, 1, 102);
  tracked_allocations = 0;
  track_allocations = true;
  const Offer measured_result = recorder.offer(measured);
  const Offer commanded_result = recorder.offer(commanded);
  track_allocations = false;

  CHECK(measured_result == Offer::Accepted);
  CHECK(commanded_result == Offer::Accepted);
  CHECK(tracked_allocations == 0);
  CHECK(recorder.stop(run_id));
  drain(recorder, writer);
  CHECK(writer.events.size() == 2);
}

class OrderedWriter final : public Writer {
public:
  explicit OrderedWriter(std::size_t count) { events.reserve(count); }

  void write(const Event &event) override {
    events.push_back(event);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      written_ = events.size();
    }
    condition_.notify_all();
  }

  void finish() override { ++finishes; }

  void wait_for_count(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    CHECK(condition_.wait_for(lock, wait_limit,
                              [&] { return written_ >= count; }));
  }

  std::vector<Event> events;
  unsigned finishes = 0;

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t written_ = 0;
};

void concurrent_ordered_stress_copies_every_event() {
  constexpr std::size_t event_count = 2048;
  constexpr std::size_t capacity = 8;
  Recorder recorder(config(capacity, event_count));
  OrderedWriter writer(event_count);
  CHECK(recorder.start(run_id, 100));

  std::thread consumer([&] { run_writer(recorder, writer); });
  std::thread producer([&] {
    for (std::size_t index = 1; index <= event_count; ++index) {
      Event event = state(index, 100 + index);
      CHECK(recorder.offer(event) == Offer::Accepted);
      event.id = 0;
      event.state_id = 0;
      event.position.fill(-1.0);
      if (index % capacity == 0)
        writer.wait_for_count(index);
    }
    CHECK(recorder.stop(run_id));
  });

  producer.join();
  consumer.join();
  const Status status = recorder.status();
  CHECK(status.phase == Phase::Complete && status.reason == Reason::Manual);
  CHECK(status.attempted == event_count && status.enqueued == event_count);
  CHECK(status.rejected == 0 && status.written == event_count);
  CHECK(status.abandoned == 0 && status.write_uncertain == 0);
  CHECK(writer.events.size() == event_count && writer.finishes == 1);
  for (std::size_t index = 0; index < event_count; ++index) {
    const Event &event = writer.events[index];
    const uint64_t id = index + 1;
    CHECK(event.id == id && event.state_id == id);
    CHECK(event.host_monotonic_ns == 100 + id);
    for (std::size_t joint = 0; joint < event.position.size(); ++joint)
      CHECK(event.position[joint] ==
            static_cast<double>(id * 16 + joint) + 0.125);
  }
}

#ifndef LBR_CAPTURE_STRESS_ONLY
void inactive_offer_does_not_hold_finalization() {
  Recorder recorder(config());
  CollectingWriter writer;
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.stop(run_id));
  hooks.arm(1);
  Offer result = Offer::Accepted;
  std::thread producer([&] { result = recorder.offer(state(1, 101)); });
  hooks.wait_until_reached();
  CHECK(recorder.status().phase == Phase::Finalizing);
  CHECK(recorder.writer_step(writer));
  CHECK(writer.finishes == 1);
  hooks.release();
  producer.join();
  CHECK(result == Offer::Inactive && recorder.status().attempted == 0);
}

void prestart_offer_cannot_be_reopened_by_start() {
  Recorder recorder(config());
  CollectingWriter writer;
  hooks.arm(1);
  Offer result = Offer::Accepted;
  std::thread producer([&] { result = recorder.offer(state(1, 101)); });
  hooks.wait_until_reached();

  CHECK(recorder.start(run_id, 100));
  hooks.release();
  producer.join();
  CHECK(result == Offer::Inactive);
  const Status armed = recorder.status();
  CHECK(armed.phase == Phase::Armed);
  CHECK(armed.attempted == 0 && armed.enqueued == 0);

  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);
  CHECK(recorder.stop(run_id));
  drain(recorder, writer);
  CHECK(writer.events.size() == 1);
}

void stop_waits_for_an_admitted_offer_to_publish() {
  Recorder recorder(config());
  CollectingWriter writer;
  CHECK(recorder.start(run_id, 100));
  hooks.arm(2);
  Offer result = Offer::Inactive;
  std::thread producer([&] { result = recorder.offer(state(1, 101)); });
  hooks.wait_until_reached();

  CHECK(recorder.stop(run_id));
  const Status stopping = recorder.status();
  CHECK(stopping.phase == Phase::Stopping);
  CHECK(stopping.reason == Reason::Manual);
  CHECK(stopping.attempted == 1 && stopping.enqueued == 0);
  CHECK(!recorder.writer_step(writer));
  CHECK(writer.finishes == 0);

  hooks.release();
  producer.join();
  CHECK(result == Offer::Accepted);
  drain(recorder, writer);
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Complete);
  CHECK(finished.attempted == 1 && finished.enqueued == 1);
  CHECK(finished.written == 1 && finished.rejected == 0);
  CHECK(writer.events.size() == 1 && writer.finishes == 1);
}

void no_data_decision_survives_concurrent_first_publication() {
  Recorder recorder(config(4, 10000, 1'000, 50));
  CollectingWriter writer;
  CHECK(recorder.start(run_id, 100));
  hooks.arm(2);
  Offer result = Offer::Inactive;
  std::thread producer([&] { result = recorder.offer(state(1, 120)); });
  hooks.wait_until_reached();

  recorder.tick(150);
  const Status stopping = recorder.status();
  CHECK(stopping.phase == Phase::Stopping);
  CHECK(stopping.reason == Reason::NoData);
  CHECK(!stopping.has_first && stopping.enqueued == 0);
  CHECK(!recorder.writer_step(writer));

  hooks.release();
  producer.join();
  CHECK(result == Offer::Accepted);
  const Status published = recorder.status();
  CHECK(published.reason == Reason::NoData && published.has_first);
  CHECK(published.first_state_ns == 120 && published.enqueued == 1);
  drain(recorder, writer);
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Failed);
  CHECK(finished.reason == Reason::NoData);
  CHECK(finished.failure_flags == failure_flag(Reason::NoData));
  CHECK(finished.written == 1 && writer.events.size() == 1);
}

void last_pop_cannot_finalize_before_active_release() {
  Recorder recorder(config());
  CollectingWriter writer;
  CHECK(recorder.start(run_id, 100));
  hooks.arm(3);
  Offer result = Offer::Inactive;
  std::thread producer([&] { result = recorder.offer(state(1, 101)); });
  hooks.wait_until_reached();

  CHECK(recorder.writer_step(writer));
  CHECK(writer.events.size() == 1);
  CHECK(recorder.stop(run_id));
  CHECK(!recorder.writer_step(writer));
  const Status blocked = recorder.status();
  CHECK(blocked.phase == Phase::Stopping);
  CHECK(!blocked.writer_done && writer.finishes == 0);

  hooks.release();
  producer.join();
  CHECK(result == Offer::Accepted);
  CHECK(recorder.writer_step(writer));
  CHECK(writer.finishes == 1);
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Complete && finished.writer_done);
  CHECK(finished.enqueued == 1 && finished.written == 1);

  const uint64_t attempted = finished.attempted;
  const uint64_t enqueued = finished.enqueued;
  hooks.arm(1);
  Offer late_result = Offer::Accepted;
  std::thread late([&] { late_result = recorder.offer(state(2, 102)); });
  hooks.wait_until_reached();
  CHECK(recorder.status().phase == Phase::Complete);
  hooks.release();
  late.join();
  CHECK(late_result == Offer::Inactive);
  const Status unchanged = recorder.status();
  CHECK(unchanged.phase == Phase::Complete);
  CHECK(unchanged.attempted == attempted && unchanged.enqueued == enqueued);
  CHECK(unchanged.written == 1 && writer.events.size() == 1);
  CHECK(!recorder.writer_step(writer));
}
#endif

struct ThrowAfterAcceptWriter final : Writer {
  std::vector<Event> accepted;
  unsigned finishes = 0;
  void write(const Event &event) override {
    accepted.push_back(event);
    throw 1;
  }
  void finish() override { ++finishes; }
};

void unknown_delivery_is_counted_once_and_never_retried() {
  Recorder recorder(config(4));
  ThrowAfterAcceptWriter writer;
  writer.accepted.reserve(1);
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);
  CHECK(recorder.offer(state(2, 102)) == Offer::Accepted);
  CHECK(recorder.offer(state(3, 103)) == Offer::Accepted);

  drain(recorder, writer);
  const Status status = recorder.status();
  CHECK(status.phase == Phase::Failed && status.writer_done);
  CHECK(status.reason == Reason::WriterFailure);
  CHECK(status.failure_flags == failure_flag(Reason::WriterFailure));
  CHECK(status.attempted == 3 && status.enqueued == 3);
  CHECK(status.rejected == 0 && status.written == 0);
  CHECK(status.write_uncertain == 1 && status.abandoned == 2);
  CHECK(status.enqueued ==
        status.written + status.write_uncertain + status.abandoned);
  CHECK(status.attempted == status.enqueued + status.rejected);
  CHECK(writer.accepted.size() == 1 && writer.accepted[0].id == 1);
  CHECK(writer.finishes == 0);
}

#ifndef LBR_CAPTURE_STRESS_ONLY
struct ThrowFinishWriter final : Writer {
  unsigned writes = 0;
  unsigned finishes = 0;
  void write(const Event &) override { ++writes; }
  void finish() override {
    ++finishes;
    throw 1;
  }
};

void status_never_reports_complete_across_finish_failure() {
  Recorder recorder(config());
  ThrowFinishWriter writer;
  CHECK(recorder.start(run_id, 100));
  CHECK(recorder.offer(state(1, 101)) == Offer::Accepted);
  CHECK(recorder.stop(run_id));
  CHECK(recorder.writer_step(writer));
  CHECK(writer.writes == 1);

  hooks.arm(4);
  Status concurrent{};
  std::thread reader([&] { concurrent = recorder.status(); });
  hooks.wait_until_reached();
  CHECK(recorder.writer_step(writer));
  CHECK(writer.finishes == 1);
  hooks.release();
  reader.join();

  CHECK(concurrent.phase == Phase::Finalizing ||
        concurrent.phase == Phase::Failed);
  CHECK(concurrent.phase != Phase::Complete);
  const Status finished = recorder.status();
  CHECK(finished.phase == Phase::Failed && finished.writer_done);
  CHECK(finished.reason == Reason::Manual);
  CHECK(finished.failure_flags == failure_flag(Reason::WriterFailure));
  CHECK(finished.written == 1 && finished.write_uncertain == 0);
}
#endif

} // namespace

int main() {
  blocked_writer_does_not_block_stop_or_status();
  blocked_writer_does_not_block_duration_expiry();
  blocked_finish_has_truthful_live_status();
  capture_offer_does_not_allocate();
  concurrent_ordered_stress_copies_every_event();
#ifndef LBR_CAPTURE_STRESS_ONLY
  inactive_offer_does_not_hold_finalization();
  prestart_offer_cannot_be_reopened_by_start();
  stop_waits_for_an_admitted_offer_to_publish();
  no_data_decision_survives_concurrent_first_publication();
  last_pop_cannot_finalize_before_active_release();
#endif
  unknown_delivery_is_counted_once_and_never_retried();
#ifndef LBR_CAPTURE_STRESS_ONLY
  status_never_reports_complete_across_finish_failure();
#endif
}
