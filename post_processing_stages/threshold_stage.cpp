/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * threshold_stage.cpp - image threshold effect
 */

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class ThresholdStage : public PostProcessingStage
{
public:
	ThresholdStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	int cutover_;
	bool invert_;
};

#define NAME "threshold"

char const *ThresholdStage::Name() const
{
	return NAME;
}

void ThresholdStage::Read(boost::property_tree::ptree const &params)
{

	cutover_ = params.get<int>("cutover", 255);
	invert_ = params.get<bool>("invert", false);
	std::cerr << "cutover = " << cutover_ << ", invert = " << invert_ << std::endl;
}

void ThresholdStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("ThresholdStage: only YUV420 format supported");
}

bool ThresholdStage::Process(CompletedRequestPtr &completed_request)
{
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	unsigned int planesz = stream_->configuration().stride * stream_->configuration().size.height;
	uint8_t *ptr = buffer.data();

	std::cerr << "plane size: " << planesz << ", buffer size: " << buffer.size() << std::endl;

	for (unsigned int i = 0; i < planesz; i++) {
		*ptr = (*ptr >= cutover_) != invert_ ? 0xFF : 0;
		ptr++;
	}
	for (unsigned int i = 0; i < planesz / 2; i++) {
		*ptr++ = 128;
	}

	printf("done\n");

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new ThresholdStage(app);
}

static RegisterStage reg(NAME, &Create);
