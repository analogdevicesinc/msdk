// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros/pluginlib/blob/1a4de29fa55173e9b897ca8ff57ebc88c047e0b3/pluginlib/include/pluginlib/impl/filesystem_helper.hpp

/*! \file filesystem_helper.hpp
 * \brief Cross-platform filesystem helper functions and additional emulation of [std::filesystem](https://en.cppreference.com/w/cpp/filesystem).
 *
 * Note: Once std::filesystem is supported on all ROS2 platforms, this class
 * can be deprecated/removed in favor of the built-in functionality.
 */

#include "rcpputils/filesystem_helper.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <system_error>
#include <vector>

#ifdef _WIN32
#  define NOMINMAX
#  define NOGDI
#  include <windows.h>
#  include <direct.h>
#  include <fileapi.h>
#  include <io.h>
#  define access _access_s
#else
#  include <dirent.h>
#  include <sys/types.h>
#  include <unistd.h>
#endif

#include "rcutils/env.h"
#include "rcpputils/split.hpp"

namespace rcpputils
{
namespace fs
{

/// \internal Returns true if the path is an absolute path with a drive letter on Windows.
static bool is_absolute_with_drive_letter(const std::string & path);

path::path(const std::string & p)  // NOLINT(runtime/explicit): this is a conversion constructor
: path_(p)
{
  std::replace(path_.begin(), path_.end(), '\\', kPreferredSeparator);
  std::replace(path_.begin(), path_.end(), '/', kPreferredSeparator);
  path_as_vector_ = split(path_, kPreferredSeparator);
}

std::string path::string() const
{
  return path_;
}

bool path::exists() const
{
  return access(path_.c_str(), 0) == 0;
}

bool path::is_directory() const noexcept
{
  struct stat stat_buffer;
  const auto rc = stat(path_.c_str(), &stat_buffer);

  if (rc != 0) {
    return false;
  }

#ifdef _WIN32
  return (stat_buffer.st_mode & S_IFDIR) == S_IFDIR;
#else
  return S_ISDIR(stat_buffer.st_mode);
#endif
}

bool path::is_regular_file() const noexcept
{
  struct stat stat_buffer;
  const auto rc = stat(path_.c_str(), &stat_buffer);

  if (rc != 0) {
    return false;
  }

#ifdef _WIN32
  return (stat_buffer.st_mode & S_IFREG) == S_IFREG;
#else
  return S_ISREG(stat_buffer.st_mode);
#endif
}

uint64_t path::file_size() const
{
  if (this->is_directory()) {
    auto ec = std::make_error_code(std::errc::is_a_directory);
    throw std::system_error{ec, "cannot get file size"};
  }

  struct stat stat_buffer;
  const auto rc = stat(path_.c_str(), &stat_buffer);

  if (rc != 0) {
    std::error_code ec{errno, std::system_category()};
    errno = 0;
    throw std::system_error{ec, "cannot get file size"};
  } else {
    return static_cast<uint64_t>(stat_buffer.st_size);
  }
}

bool path::empty() const
{
  return path_.empty();
}

bool path::is_absolute() const
{
  return path_.size() > 0 &&
         (path_[0] == kPreferredSeparator ||
         is_absolute_with_drive_letter(path_));
}

std::vector<std::string>::const_iterator path::cbegin() const
{
  return path_as_vector_.cbegin();
}

std::vector<std::string>::const_iterator path::cend() const
{
  return path_as_vector_.cend();
}

path path::parent_path() const
{
  // Edge case: empty path
  if (this->empty()) {
    return path("");
  }

  // Edge case: if path only consists of one part, then return '.' or '/'
  //            depending if the path is absolute or not
  if (1u == path_as_vector_.size()) {
    if (this->is_absolute()) {
      // Windows is tricky, since an absolute path may start with 'C:\\' or '\\'
      if (is_absolute_with_drive_letter(path_)) {
        return path(path_as_vector_[0] + kPreferredSeparator);
      }
      return path(std::string(1, kPreferredSeparator));
    }
    return path(".");
  }

  // Edge case: with a path 'C:\\foo' we want to return 'C:\\' not 'C:'
  // Don't drop the root directory from an absolute path on Windows starting with a letter drive
  if (2u == path_as_vector_.size() && is_absolute_with_drive_letter(path_)) {
    return path(path_as_vector_[0] + kPreferredSeparator);
  }

  path parent;
  for (auto it = this->cbegin(); it != --this->cend(); ++it) {
    if (parent.empty() && (!this->is_absolute() || is_absolute_with_drive_letter(path_))) {
      // This handles the case where we are dealing with a relative path or
      // the Windows drive letter; in both cases we don't want a separator at
      // the beginning, so just copy the piece directly.
      parent = *it;
    } else {
      parent /= *it;
    }
  }
  return parent;
}

path path::filename() const
{
  return path_.empty() ? path() : *--this->cend();
}

path path::extension() const
{
  const char * delimiter = ".";
  auto split_fname = rcpputils::split(this->string(), *delimiter);
  return split_fname.size() == 1 ? path("") : path("." + split_fname.back());
}

path path::operator/(const std::string & other) const
{
  return this->operator/(path(other));
}

path & path::operator/=(const std::string & other)
{
  this->operator/=(path(other));
  return *this;
}

path path::operator/(const path & other) const
{
  return path(*this).operator/=(other);
}

path & path::operator/=(const path & other)
{
  if (other.is_absolute()) {
    this->path_ = other.path_;
    this->path_as_vector_ = other.path_as_vector_;
  } else {
    if (this->path_.empty() || this->path_[this->path_.length() - 1] != kPreferredSeparator) {
      // This ensures that we don't put duplicate separators into the path;
      // this can happen, for instance, on absolute paths where the first
      // item in the vector is the empty string.
      this->path_ += kPreferredSeparator;
    }
    this->path_ += other.string();
    this->path_as_vector_.insert(
      std::end(this->path_as_vector_),
      std::begin(other.path_as_vector_), std::end(other.path_as_vector_));
  }
  return *this;
}

static bool is_absolute_with_drive_letter(const std::string & path)
{
  (void)path;  // Maybe unused
#ifdef _WIN32
  if (path.empty()) {
    return false;
  }
  return 0 == path.compare(1, 2, ":\\");
#else
  return false;  // only Windows contains absolute paths starting with drive letters
#endif
}

bool is_regular_file(const path & p) noexcept
{
  return p.is_regular_file();
}

bool is_directory(const path & p) noexcept
{
  return p.is_directory();
}

uint64_t file_size(const path & p)
{
  return p.file_size();
}

bool exists(const path & path_to_check)
{
  return path_to_check.exists();
}

path temp_directory_path()
{
#ifdef _WIN32
#ifdef UNICODE
#error "rcpputils::fs does not support Unicode paths"
#endif
  TCHAR temp_path[MAX_PATH];
  DWORD size = GetTempPathA(MAX_PATH, temp_path);
  if (size > MAX_PATH || size == 0) {
    std::error_code ec(static_cast<int>(GetLastError()), std::system_category());
    throw std::system_error(ec, "cannot get temporary directory path");
  }
  temp_path[size] = '\0';
#else
  const char * temp_path = NULL;
  const char * ret_str = rcutils_get_env("TMPDIR", &temp_path);
  if (NULL != ret_str || *temp_path == '\0') {
    temp_path = "/tmp";
  }
#endif
  return path(temp_path);
}

path create_temp_directory(const std::string & base_name, const path & parent_path)
{
  const auto template_path = base_name + "XXXXXX";
  std::string full_template_str = (parent_path / template_path).string();
  if (!create_directories(parent_path)) {
    std::error_code ec{errno, std::system_category()};
    errno = 0;
    throw std::system_error(ec, "could not create the parent directory");
  }

#ifdef _WIN32
  errno_t errcode = _mktemp_s(&full_template_str[0], full_template_str.size() + 1);
  if (errcode) {
    std::error_code ec(static_cast<int>(errcode), std::system_category());
    throw std::system_error(ec, "could not format the temp directory name template");
  }
  const path final_path{full_template_str};
  if (!create_directories(final_path)) {
    std::error_code ec(static_cast<int>(GetLastError()), std::system_category());
    throw std::system_error(ec, "could not create the temp directory");
  }
#else
  const char * dir_name = mkdtemp(&full_template_str[0]);
  if (dir_name == nullptr) {
    std::error_code ec{errno, std::system_category()};
    errno = 0;
    throw std::system_error(ec, "could not format or create the temp directory");
  }
  const path final_path{dir_name};
#endif

  return final_path;
}

path current_path()
{
#ifdef _WIN32
#ifdef UNICODE
#error "rcpputils::fs does not support Unicode paths"
#endif
  char cwd[MAX_PATH];
  if (nullptr == _getcwd(cwd, MAX_PATH)) {
#else
  char cwd[PATH_MAX];
  if (nullptr == getcwd(cwd, PATH_MAX)) {
#endif
    std::error_code ec{errno, std::system_category()};
    errno = 0;
    throw std::system_error{ec, "cannot get current working directory"};
  }

  return path(cwd);
}

bool create_directories(const path & p)
{
  path p_built;
  int status = 0;

  for (auto it = p.cbegin(); it != p.cend() && status == 0; ++it) {
    if (!p_built.empty() || it->empty()) {
      p_built /= *it;
    } else {
      p_built = *it;
    }
    if (!p_built.exists()) {
#ifdef _WIN32
      status = _mkdir(p_built.string().c_str());
#else
      status = mkdir(p_built.string().c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
#endif
      if (status == -1 && errno == EEXIST) {
        status = 0;
      }
    }
  }
  return status == 0 && p_built.is_directory();
}

bool remove(const path & p)
{
#ifdef _WIN32
  struct _stat s;
  if (_stat(p.string().c_str(), &s) == 0) {
    if (s.st_mode & S_IFDIR) {
      return _rmdir(p.string().c_str()) == 0;
    }
    if (s.st_mode & S_IFREG) {
      return ::remove(p.string().c_str()) == 0;
    }
  }
  return false;
#else
  return ::remove(p.string().c_str()) == 0;
#endif
}

bool remove_all(const path & p)
{
  if (!is_directory(p)) {
    return rcpputils::fs::remove(p);
  }

#ifdef _WIN32
  // We need a string of type PCZZTSTR, which is a double null terminated char ptr
  size_t length = p.string().size();
  TCHAR * temp_dir = new TCHAR[length + 2];
  memcpy(temp_dir, p.string().c_str(), length);
  temp_dir[length] = '\0';
  temp_dir[length + 1] = '\0';  // double null terminated

  SHFILEOPSTRUCT file_options;
  file_options.hwnd = nullptr;
  file_options.wFunc = FO_DELETE;  // delete (recursively)
  file_options.pFrom = temp_dir;
  file_options.pTo = nullptr;
  file_options.fFlags = FOF_NOCONFIRMATION | FOF_SILENT;  // do not prompt user
  file_options.fAnyOperationsAborted = FALSE;
  file_options.lpszProgressTitle = nullptr;
  file_options.hNameMappings = nullptr;

  auto ret = SHFileOperation(&file_options);
  delete[] temp_dir;

  return 0 == ret && false == file_options.fAnyOperationsAborted;
#else
  DIR * dir = opendir(p.string().c_str());
  struct dirent * directory_entry;
  while ((directory_entry = readdir(dir)) != nullptr) {
    // Make sure to not call ".." or "." entries in directory (might delete everything)
    if (strcmp(directory_entry->d_name, ".") != 0 && strcmp(directory_entry->d_name, "..") != 0) {
      auto sub_path = rcpputils::fs::path(p) / directory_entry->d_name;
      // if directory, call recursively
      if (sub_path.is_directory() && !rcpputils::fs::remove_all(sub_path)) {
        return false;
        // if not, call regular remove
      } else if (!rcpputils::fs::remove(sub_path)) {
        return false;
      }
    }
  }
  closedir(dir);
  // directory is empty now, call remove
  rcpputils::fs::remove(p);
  return !rcpputils::fs::exists(p);
#endif
}

path remove_extension(const path & file_path, int n_times)
{
  path new_path(file_path);
  for (int i = 0; i < n_times; i++) {
    const auto new_path_str = new_path.string();
    const auto last_dot = new_path_str.find_last_of('.');
    if (last_dot == std::string::npos) {
      return new_path;
    }
    new_path = path(new_path_str.substr(0, last_dot));
  }
  return new_path;
}

bool operator==(const path & a, const path & b)
{
  return a.string() == b.string();
}

bool operator!=(const path & a, const path & b)
{
  return !(a == b);
}

std::ostream & operator<<(std::ostream & os, const path & p)
{
  os << p.string();
  return os;
}

}  // namespace fs
}  // namespace rcpputils
