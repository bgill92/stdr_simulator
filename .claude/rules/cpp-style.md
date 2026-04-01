---
paths:
  - "*.cpp"
  - "*.hpp"
---

# C++ Style — Project-Specific Rules

## Modern C++20

- `[[nodiscard]]` on all functions returning values.
- Structured bindings for multiple return values.
- Respect `[[nodiscard]]` — it signals the return value matters. When you genuinely need to discard it (e.g., in test setup), use `std::ignore = func();`.
- Use `auto` only when it makes code clearer or safer per [Abseil Tip 232](https://abseil.io/tips/232). When in doubt, spell out the type.
- `auto` is appropriate for: structured bindings (`const auto& [k, v] = ...`), iterators where the container is visible nearby, factory calls where the type appears on the right (`std::make_unique<T>`), and `constexpr auto` for string literals and numeric constants.
- Do not use `auto` with Eigen types. Eigen uses expression templates and intermediate proxy types, so `auto` captures lazy expressions instead of concrete values — risking dangling references and subtle bugs. Always spell out the Eigen type: `const Eigen::Vector3d v = rotation.col(2);` not `const auto v = rotation.col(2);`.
- Do not use `auto` when the type is not obvious from context or when it hides important semantics like constness, pointer vs. value, or whether a copy occurs.

## Conventions

- If a variable can be made `const`, make it `const`. Do not make member variables `const` per [Abseil Tip 177](https://abseil.io/tips/177).
- Do not use `std::bind`. Use a lambda or `std::bind_front` per [Abseil Tip 108](https://abseil.io/tips/108).
- Prefer `.hpp` + `.cpp` over header-only inline. Forward-declare in headers when possible.
- `tl::expected<T, std::string>` for recoverable errors. Exceptions only for truly exceptional cases.
- Avoid `const_cast` — copy the data or refactor the API instead.
- Thread safety is the **default expectation** — only document (`@warning`) when NOT thread-safe.

## Performance

- Prefer `std::execution::par_unseq` for data-parallel transforms on large collections.
- Profile before optimizing — measure, don't guess.
- Benchmark performance-critical code paths (see testing.md for CMake setup).

## Code Comments

- Document WHY, not WHAT. Doxygen `/** ... */` for public APIs.
- No internal tracking references (bead IDs, ticket numbers) in code comments.
- End human-readable, full-sentence comments with a period. Do not alter tool-control comments (e.g., `// NOLINT...`, `// clang-format off`) or structured tags (e.g., Doxygen `@param` lines, `// TODO`).
