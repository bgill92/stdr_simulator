#pragma once

#include <string>
#include <vector>

namespace stdr_gui
{

/** @brief Result of listing a directory for YAML files. */
struct DirectoryListing
{
  std::vector<std::string> directories;
  std::vector<std::string> yaml_files;
};

/** @brief List directories and YAML files (.yaml, .yml) in the given path.
 *
 *  Results are sorted alphabetically. Returns empty lists on error. */
[[nodiscard]] DirectoryListing list_yaml_directory(const std::string& path);

/** @brief Reason the file dialog was opened. */
enum class FileDialogPurpose
{
  kLoadMap,
  kLoadRobot,
};

/** @brief Simple ImGui-based file browser for loading maps and robot configs.
 *
 *  @warning Not thread-safe. Must be called from the render thread only. */
class FileDialog
{
public:
  /** @brief Open the dialog for a specific purpose. */
  void open(FileDialogPurpose purpose, const std::string& start_dir = ".");

  /** @brief Render the dialog. Returns true if a file was selected this frame. */
  [[nodiscard]] bool render();

  /** @brief Get the selected file path (valid after render() returns true). */
  [[nodiscard]] const std::string& selected_path() const;

  /** @brief Get the purpose this dialog was opened for. */
  [[nodiscard]] FileDialogPurpose purpose() const;

  /** @brief Check if the dialog is currently open. */
  [[nodiscard]] bool is_open() const;

private:
  void refresh_entries();

  bool open_{ false };
  FileDialogPurpose purpose_{ FileDialogPurpose::kLoadMap };
  std::string current_dir_;
  std::string selected_path_;
  std::vector<std::string> dir_entries_;
  std::vector<std::string> file_entries_;
  int selected_index_{ -1 };
};

}  // namespace stdr_gui
