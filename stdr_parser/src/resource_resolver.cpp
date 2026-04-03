#include <stdr_parser/resource_resolver.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>

namespace stdr_parser {

tl::expected<std::string, std::string> get_resources_dir()
{
  try {
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("stdr_resources");
    return share_dir;
  } catch (const std::exception& e) {
    return tl::unexpected(
        std::string("Failed to find stdr_resources package: ") + e.what());
  }
}

tl::expected<std::string, std::string> resolve_resource_path(
    const std::string& relative_path)
{
  const auto base = get_resources_dir();
  if (!base) return tl::unexpected(base.error());

  const std::filesystem::path full =
      std::filesystem::path(*base) / "resources" / relative_path;

  if (!std::filesystem::exists(full)) {
    return tl::unexpected("Resource file not found: " + full.string());
  }
  return full.string();
}

tl::expected<std::string, std::string> get_specifications_dir()
{
  const auto base = get_resources_dir();
  if (!base) return tl::unexpected(base.error());

  const std::filesystem::path specs =
      std::filesystem::path(*base) / "resources" / "specifications";

  if (!std::filesystem::is_directory(specs)) {
    return tl::unexpected("Specifications directory not found: " + specs.string());
  }
  return specs.string();
}

}  // namespace stdr_parser
