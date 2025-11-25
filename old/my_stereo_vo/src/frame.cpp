#include "my_stereo_vo/frame.hpp"
namespace StereoSLAM {

int32_t Frame::nextFrameId = 0;

Frame::Frame() : frameId(++Frame::nextFrameId) {}
void Frame::setKeyFrame() { isKeyFrame_ = true; }

} // namespace StereoSLAM