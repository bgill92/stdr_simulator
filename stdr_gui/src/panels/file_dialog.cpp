#include <stdr_gui/panels/file_dialog.hpp>

#include <imgui.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace stdr_gui
{

void FileDialog::open(FileDialogPurpose purpose, const std::string& start_dir)
{
  purpose_ = purpose;
  current_dir_ = start_dir;
  selected_path_.clear();
  selected_index_ = -1;
  open_ = true;
  refresh_entries();
}

bool FileDialog::render()
{
  if (!open_)
  {
    return false;
  }

  const char* title = (purpose_ == FileDialogPurpose::kLoadMap) ? "Load Map##FileDialog" : "Load Robot##FileDialog";

  ImGui::OpenPopup(title);

  bool file_selected = false;

  const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(600.0f, 400.0f), ImGuiCond_Appearing);

  if (ImGui::BeginPopupModal(title, &open_))
  {
    // Show current directory path.
    ImGui::TextUnformatted(current_dir_.c_str());
    ImGui::Separator();

    if (ImGui::BeginChild("##entries", ImVec2(0.0f, -40.0f)))
    {
      // Parent directory navigation.
      if (ImGui::Selectable("../", false))
      {
        const std::filesystem::path parent = std::filesystem::path(current_dir_).parent_path();
        current_dir_ = parent.string();
        selected_index_ = -1;
        refresh_entries();
      }

      // Directories first.
      for (const std::string& dir : dir_entries_)
      {
        const std::string label = dir + "/";
        if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick))
        {
          if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
          {
            current_dir_ = (std::filesystem::path(current_dir_) / dir).string();
            selected_index_ = -1;
            refresh_entries();
          }
        }
      }

      // YAML files.
      for (int i = 0; i < static_cast<int>(file_entries_.size()); ++i)
      {
        const bool is_selected = (selected_index_ == i);
        if (ImGui::Selectable(file_entries_[i].c_str(), is_selected))
        {
          selected_index_ = i;
          selected_path_ = (std::filesystem::path(current_dir_) / file_entries_[i]).string();
        }
      }
    }
    ImGui::EndChild();

    ImGui::Separator();

    const bool has_selection = (selected_index_ >= 0);
    if (!has_selection)
    {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Open", ImVec2(80.0f, 0.0f)))
    {
      file_selected = true;
      open_ = false;
      ImGui::CloseCurrentPopup();
    }
    if (!has_selection)
    {
      ImGui::EndDisabled();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80.0f, 0.0f)))
    {
      open_ = false;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }

  return file_selected;
}

const std::string& FileDialog::selected_path() const
{
  return selected_path_;
}

FileDialogPurpose FileDialog::purpose() const
{
  return purpose_;
}

bool FileDialog::is_open() const
{
  return open_;
}

DirectoryListing list_yaml_directory(const std::string& path)
{
  DirectoryListing result;
  std::error_code ec;
  const std::filesystem::directory_iterator it(path, ec);
  if (ec)
  {
    return result;
  }
  for (const std::filesystem::directory_entry& entry : it)
  {
    if (entry.is_directory())
    {
      result.directories.push_back(entry.path().filename().string());
    }
    else if (entry.is_regular_file())
    {
      const std::string ext = entry.path().extension().string();
      if (ext == ".yaml" || ext == ".yml")
      {
        result.yaml_files.push_back(entry.path().filename().string());
      }
    }
  }
  std::ranges::sort(result.directories);
  std::ranges::sort(result.yaml_files);
  return result;
}

void FileDialog::refresh_entries()
{
  const DirectoryListing listing = list_yaml_directory(current_dir_);
  dir_entries_ = listing.directories;
  file_entries_ = listing.yaml_files;
}

}  // namespace stdr_gui
