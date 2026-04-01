---
paths:
  - "test/**/*.cpp"
  - "test/**/*.hpp"
  - "test_*.cpp"
  - "src/**/CMakeLists.txt"
---

# C++ Testing Rules

## Naming

Use PascalCase for test suite and test names: `TEST(MyTestSuite, MyTestName)` not `TEST(my_test_suite, my_test_name)`.

## Fixture Initialization

Prefer constructor over `SetUp()`. `SetUp()` only needed for virtual methods or `ASSERT_*`.

## Bounds Checking

Always `ASSERT_THAT(vec, SizeIs(N))` before `vec[0]` — project convention for variable-length containers.

## BehaviorContext in Tests

**Always** set `tf2_spin_thread=false` to avoid deadlocks. If tf2 lookups are needed, manually spin the node/executor instead.

## Mock Testing

Use free helper functions to construct objects from `unique_ptr` mocks. Never store raw pointers to mocks in a fixture. When mock expectations are needed (`EXPECT_CALL`), use a helper struct that owns `unique_ptr` members before moving them into the object under test.

## Test Helpers and Matchers

Only create helpers and matchers that are **trivially correct** — so simple they cannot reasonably be wrong. If a helper needs its own tests to verify correctness, it's too complex.

Good: a one-line wrapper around a constructor or `std::ranges::minmax_element`.
Bad: a helper that calculates expected values with 20 lines of math.

## Parameterization

Use `TestWithParam` when 3+ tests share the same structure with different inputs. Don't parameterize tests with different assertion logic — use separate `TEST` functions instead.

When NOT to parameterize:

- Tests with significantly different logic (use separate tests)
- When it reduces clarity (names become too generic)
- Single test case (no duplication to eliminate)

## Test Ordering

Order tests from simplest to most complex: error validation first, then trivial cases, then complex scenarios. This builds confidence incrementally.

## Avoid Redundant Validation

Don't re-verify conditions already validated by earlier tests in the same file. If `ValidParametersSucceed` already proved `(1000, 1000, 5)` works, later tests can call `.value()` directly without an `ASSERT_THAT(result, IsOk())` check.

## Benchmarks (CMake)

Use `ament_add_google_benchmark` — never hardcode paths to benchmark libraries, set RPATH manually, use `google_benchmark_vendor`, or link version-specific `.so` files.
