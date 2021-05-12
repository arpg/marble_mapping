/**
* marble_mapping: A Tool to serve and merge octomaps
* (inspired by octomap_server)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2012.
* @author D. Riley, Copyright (C) 2020.
* License: BSD
*/

/*
 * Original work Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * Modified work Copyright 2020 Dan Riley
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <MarbleMapping.h>

#define USAGE "\nUSAGE: marble_mapping <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace marble_mapping;

int main(int argc, char** argv){
  ros::init(argc, argv, "marble_mapping");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");
  std::string mapFilename(""), mapFilenameParam("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  MarbleMapping server(private_nh, nh);
  ros::spinOnce();

  if (argc == 2){
    mapFilename = std::string(argv[1]);
  }

  if (private_nh.getParam("map_file", mapFilenameParam)) {
    if (mapFilename != "") {
      ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
    } else {
      mapFilename = mapFilenameParam;
    }
  }

  if (mapFilename != "") {
    if (!server.openFile(mapFilename)){
      ROS_ERROR("Could not open file %s", mapFilename.c_str());
      exit(1);
    }
  }

  try{
    // Use separate threads for the map publishers
    ros::AsyncSpinner async_spinner(2, &server.pub_queue);
    async_spinner.start();
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("marble_mapping exception: %s", e.what());
    return -1;
  }

  return 0;
}
