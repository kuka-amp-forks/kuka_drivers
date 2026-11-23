// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "kuka_rsi_driver/rsi_xml_configuration_parser.hpp"

using kuka::external::control::kss::Configuration;
using kuka::external::control::kss::MotionStateSignalType;
using kuka_rsi_driver::RsiXmlConfigurationParser;

namespace
{

constexpr std::size_t kJoints = 6;

class RsiXmlConfigurationParserTest : public ::testing::Test
{
protected:
  bool Load(const std::string & path, std::size_t joints = kJoints)
  {
    return parser_.Load(path, joints, 0, config_);
  }

  std::string WriteFile(const std::string & text)
  {
    const std::string path = ::testing::TempDir() + "rsi_xml_config_test.yaml";
    std::ofstream(path) << text;
    return path;
  }

  RsiXmlConfigurationParser parser_{rclcpp::get_logger("rsi_xml_configuration_parser_test")};
  Configuration config_;
};

}  // namespace

TEST_F(RsiXmlConfigurationParserTest, LoadsTheShippedExample)
{
  ASSERT_TRUE(Load(RSI_XML_CONFIG_EXAMPLE));

  ASSERT_TRUE(config_.motion_state_xml_config.has_value());
  const auto & fields = config_.motion_state_xml_config->joint_fields;
  ASSERT_EQ(fields.size(), kJoints);
  for (std::size_t i = 0; i < kJoints; i++)
  {
    EXPECT_EQ(fields[i].joint_identifier, "joint_" + std::to_string(i + 1));
    EXPECT_EQ(fields[i].signal_type, MotionStateSignalType::POSITION);
    EXPECT_EQ(fields[i].xml_element, "AIPos");
    EXPECT_EQ(fields[i].xml_attribute, "A" + std::to_string(i + 1));
  }
  EXPECT_EQ(config_.motion_state_xml_config->cartesian.xml_element, "RIst");
  EXPECT_TRUE(config_.control_signal_xml_config.has_value());
}

TEST_F(RsiXmlConfigurationParserTest, RejectsAJointCountMismatch)
{
  EXPECT_FALSE(Load(RSI_XML_CONFIG_EXAMPLE, kJoints + 1));
}

TEST_F(RsiXmlConfigurationParserTest, RejectsAMissingFile)
{
  EXPECT_FALSE(Load(::testing::TempDir() + "no_such_rsi_xml_config.yaml"));
}

TEST_F(RsiXmlConfigurationParserTest, RejectsAFileWithoutTheTopLevelKey)
{
  EXPECT_FALSE(Load(WriteFile("motion_state: {}\n")));
}

TEST_F(RsiXmlConfigurationParserTest, RejectsMalformedYaml)
{
  EXPECT_FALSE(Load(WriteFile("rsi_xml_config: [\n")));
}
