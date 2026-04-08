#include <stdr_gui/panels/file_dialog.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace stdr_gui
{
namespace
{

using ::testing::ElementsAre;
using ::testing::IsEmpty;
using ::testing::UnorderedElementsAre;

// ---- Test fixture -----------------------------------------------------------

class ListYamlDirectoryTest : public ::testing::Test
{
public:
  ListYamlDirectoryTest()
  {
    // Use a unique temp directory per test to avoid interference.
    temp_dir_ = std::filesystem::temp_directory_path() / "stdr_gui_test_file_dialog";
    std::filesystem::remove_all(temp_dir_);
    std::filesystem::create_directories(temp_dir_);
  }

  ~ListYamlDirectoryTest() override
  {
    std::filesystem::remove_all(temp_dir_);
  }

  // Create an empty file at temp_dir_ / filename.
  void touch(const std::string& filename) const
  {
    std::ofstream{ temp_dir_ / filename };
  }

  // Create a subdirectory at temp_dir_ / name.
  void mkdir(const std::string& name) const
  {
    std::filesystem::create_directories(temp_dir_ / name);
  }

  std::filesystem::path temp_dir_;
};

// ---- Error cases ------------------------------------------------------------

TEST_F(ListYamlDirectoryTest, NonExistentDirectoryReturnsEmpty)
{
  const std::string bad_path = (temp_dir_ / "does_not_exist").string();
  const DirectoryListing result = list_yaml_directory(bad_path);
  EXPECT_THAT(result.directories, IsEmpty());
  EXPECT_THAT(result.yaml_files, IsEmpty());
}

// ---- Trivial cases ----------------------------------------------------------

TEST_F(ListYamlDirectoryTest, EmptyDirectoryReturnsEmpty)
{
  const DirectoryListing result = list_yaml_directory(temp_dir_.string());
  EXPECT_THAT(result.directories, IsEmpty());
  EXPECT_THAT(result.yaml_files, IsEmpty());
}

// ---- Filtering --------------------------------------------------------------

TEST_F(ListYamlDirectoryTest, FiltersYamlFiles)
{
  touch("foo.yaml");
  touch("bar.yml");
  touch("baz.txt");
  touch("qux.json");

  const DirectoryListing result = list_yaml_directory(temp_dir_.string());
  EXPECT_THAT(result.yaml_files, UnorderedElementsAre("foo.yaml", "bar.yml"));
  EXPECT_THAT(result.directories, IsEmpty());
}

TEST_F(ListYamlDirectoryTest, ListsDirectories)
{
  mkdir("subA");
  mkdir("subB");

  const DirectoryListing result = list_yaml_directory(temp_dir_.string());
  EXPECT_THAT(result.directories, UnorderedElementsAre("subA", "subB"));
  EXPECT_THAT(result.yaml_files, IsEmpty());
}

// ---- Sorting ----------------------------------------------------------------

TEST_F(ListYamlDirectoryTest, ResultsAreSorted)
{
  touch("z.yaml");
  touch("a.yaml");
  touch("m.yaml");

  const DirectoryListing result = list_yaml_directory(temp_dir_.string());
  EXPECT_THAT(result.yaml_files, ElementsAre("a.yaml", "m.yaml", "z.yaml"));
}

// ---- Mixed content ----------------------------------------------------------

TEST_F(ListYamlDirectoryTest, MixedContent)
{
  touch("robot.yaml");
  touch("map.yml");
  touch("readme.txt");
  mkdir("configs");
  mkdir("maps");

  const DirectoryListing result = list_yaml_directory(temp_dir_.string());
  EXPECT_THAT(result.directories, ElementsAre("configs", "maps"));
  EXPECT_THAT(result.yaml_files, ElementsAre("map.yml", "robot.yaml"));
}

}  // namespace
}  // namespace stdr_gui
