/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace ocs2 {
namespace robotic_assets {

/** Locate installed assets through the sourced ROS overlay. */
inline std::string getPath() {
  const char* overridePath = std::getenv("OCS2_ROBOTIC_ASSETS_PATH");
  if (overridePath && *overridePath) return overridePath;

  const char* env = std::getenv("AMENT_PREFIX_PATH");
  std::string prefixes = env ? env : "";
  std::size_t start = 0;
  while (start <= prefixes.size()) {
    const auto end = prefixes.find(':', start);
    const auto prefix = prefixes.substr(start, end - start);
    if (!prefix.empty()) {
      const auto share = prefix + "/share/ocs2_robotic_assets";
      struct stat info;
      if (::stat((share + "/resources").c_str(), &info) == 0 && S_ISDIR(info.st_mode)) return share;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  throw std::runtime_error("ocs2_robotic_assets not found; source the ROS workspace setup.bash");
}

}  // namespace robotic_assets
}  // namespace ocs2
