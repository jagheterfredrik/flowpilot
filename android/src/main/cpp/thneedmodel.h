#pragma once

#include <string>

#include "runmodel.h"
#include "thneed.h"
#include "messaging/messaging.h"

class ThneedModel : public RunModel {
public:
  ThneedModel(byte* modelData, float *_output, size_t _output_size, int runtime, bool use_tf8 = false, cl_context context = NULL);
  void *getCLBuffer(const std::string name);
  void execute();
private:
  Thneed *thneed = NULL;
  PubMaster pm({"modelV2", "cameraOdometry"});
  bool recorded;
  float *output;
};
