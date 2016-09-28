/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/tnm067lab1/processors/imageupsampler.h>

namespace inviwo {

	// The Class Identifier has to be globally unique. Use a reverse DNS naming
	// scheme
	const ProcessorInfo ImageUpsampler::processorInfo_{
		"org.inviwo.imageupsampler",  // Class identifier
		"Image Upsampler",            // Display name
		"TNM067",                     // Category
		CodeState::Experimental,      // Code state
		Tags::None,                   // Tags
	};
	const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

	ImageUpsampler::ImageUpsampler()
		: Processor()
		, imageInport_("inport", false)
		, grayscaleImageOutport_("outport", DataFloat32::get(), false)
		, colorImageOutport_("coloroutport", DataUInt8::get(), false)
		, samplerSize_("samplerSize", "Sampler Size")
		, interpolationMethod_("interpolationMethod", "Interpolation Method")
		, pixelIntensityScaleFactor_("pixelIntensityScaleFactor", "IntensityScale Factor", 1.0, 0.001,
			2.0, 0.001) {
		addPort(imageInport_);
		addPort(grayscaleImageOutport_);
		addPort(colorImageOutport_);

		samplerSize_.addOption("1x1", "1X1", 1);
		samplerSize_.addOption("2x2", "2X2", 2);
		samplerSize_.addOption("4x4", "4X4", 4);
		samplerSize_.addOption("8x8", "8X8", 8);

		addProperty(samplerSize_);

		interpolationMethod_.addOption("piecewiseconstant", "Piecewise Constant", 1);
		interpolationMethod_.addOption("bilinear", "Bilinear", 2);
		interpolationMethod_.addOption("quadratic", "Quadratic", 3);
		interpolationMethod_.addOption("barycentric", "Barycentric", 4);
		interpolationMethod_.setCurrentStateAsDefault();
		addProperty(interpolationMethod_);

		addProperty(pixelIntensityScaleFactor_);
	}

	void ImageUpsampler::process() {
		auto imageIn = imageInport_.getData();
		auto inSize = imageInport_.getData()->getDimensions();
		auto outDim = inSize * size2_t(samplerSize_.get());

		auto grayscaleOutputImage =
			std::make_shared<Image>(outDim, imageInport_.getData()->getDataFormat());
		imageInport_.getData()->copyRepresentationsTo(grayscaleOutputImage.get());

		switch (interpolationMethod_.get()) {
		case 1:
			piecewiseConstant(imageIn.get(), grayscaleOutputImage.get());
			break;
		case 2:
			bilinearInterpolation(imageIn.get(), grayscaleOutputImage.get());
			break;
		case 3:
			quadraticInterpolation(imageIn.get(), grayscaleOutputImage.get());
			break;
		case 4:
			barycentricInterpolation(imageIn.get(), grayscaleOutputImage.get());
			break;
		};

		auto colorscaleImage = std::make_shared<Image>(outDim, DataVec4UInt8::get());
		applyColorMap(grayscaleOutputImage.get(), colorscaleImage.get());

		grayscaleImageOutport_.setData(grayscaleOutputImage);
		colorImageOutport_.setData(colorscaleImage);
	}

	void ImageUpsampler::piecewiseConstant(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto sampleSize = samplerSize_.get();
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {

				float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
				float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

				x *= static_cast<float>(inputSize.x) - 1.f;
				y *= static_cast<float>(inputSize.y) - 1.f;

				size2_t P0(round(x), round(y));

				P0 = glm::clamp(P0, size2_t(0), inputSize - size_t(1));

				// get pixel from input image at pixel coordinate mappedIndex
				auto pixel_intensity = in_img->getAsNormalizedDouble(P0);

				out_img->setFromNormalizedDVec4(
					size2_t(i, j),
					dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
			}
		}
	}

	void ImageUpsampler::bilinearInterpolation(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto sampleSize = samplerSize_.get();
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {

				float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
				float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

				x *= static_cast<float>(inputSize.x) - 1.f;
				y *= static_cast<float>(inputSize.y) - 1.f;

				size2_t P0(floor(x), floor(y));
				size2_t P1(floor(x) + 1.f, floor(y));
				size2_t P2(floor(x) + 1.f, floor(y) + 1.f);
				size2_t P3(floor(x), floor(y) + 1.f);

				double u = (P2.x - static_cast<double>(x)) / (P2.x - P0.x);
				double v = (P2.y - static_cast<double>(y)) / (P2.y - P0.y);

				P0 = glm::clamp(P0, size2_t(0), inputSize - size_t(1));
				P1 = glm::clamp(P1, size2_t(0), inputSize - size_t(1));
				P2 = glm::clamp(P2, size2_t(0), inputSize - size_t(1));
				P3 = glm::clamp(P3, size2_t(0), inputSize - size_t(1));

				double f0 = in_img->getAsNormalizedDouble(P0);
				double f1 = in_img->getAsNormalizedDouble(P1);
				double f2 = in_img->getAsNormalizedDouble(P2);
				double f3 = in_img->getAsNormalizedDouble(P3);

				double a = (1 - u)*f1 + u*f0;
				double b = (1 - u)*f2 + u*f3;

				double pixel_intensity = (1 - v)*b + v*a;

				out_img->setFromNormalizedDVec4(
					size2_t(i, j),
					dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image */
			}
		}
	}

	void ImageUpsampler::quadraticInterpolation(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto sampleSize = samplerSize_.get();
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		ivec2 clamper = inputSize - size_t(1);

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {

				float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
				float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

				x *= static_cast<float>(inputSize.x) - 1.f;
				y *= static_cast<float>(inputSize.y) - 1.f;

				auto rx = round(x);
				auto ry = round(y);

				ivec2 P0(rx - 1.f, ry - 1.f);
				ivec2 P1 = P0 + ivec2(1, 0);
				ivec2 P2 = P0 + ivec2(2, 0);
				ivec2 P3 = P0 + ivec2(0, 1);
				ivec2 P4 = P0 + ivec2(1, 1);
				ivec2 P5 = P0 + ivec2(2, 1);
				ivec2 P6 = P0 + ivec2(0, 2);
				ivec2 P7 = P0 + ivec2(1, 2);
				ivec2 P8 = P0 + ivec2(2, 2);

				double u = (static_cast<double>(x) - P0.x) / (P2.x - P0.x);
				double v = (static_cast<double>(y) - P0.y) / (P7.y - P0.y);

				P0 = glm::clamp(P0, ivec2(0), clamper);
				P1 = glm::clamp(P1, ivec2(0), clamper);
				P2 = glm::clamp(P2, ivec2(0), clamper);
				P3 = glm::clamp(P3, ivec2(0), clamper);
				P4 = glm::clamp(P4, ivec2(0), clamper);
				P5 = glm::clamp(P5, ivec2(0), clamper);
				P6 = glm::clamp(P6, ivec2(0), clamper);
				P7 = glm::clamp(P7, ivec2(0), clamper);
				P8 = glm::clamp(P8, ivec2(0), clamper);

				double f0 = in_img->getAsNormalizedDouble(P0);
				double f1 = in_img->getAsNormalizedDouble(P1);
				double f2 = in_img->getAsNormalizedDouble(P2);
				double f3 = in_img->getAsNormalizedDouble(P3);
				double f4 = in_img->getAsNormalizedDouble(P4);
				double f5 = in_img->getAsNormalizedDouble(P5);
				double f6 = in_img->getAsNormalizedDouble(P6);
				double f7 = in_img->getAsNormalizedDouble(P7);
				double f8 = in_img->getAsNormalizedDouble(P8);


				double a = (1.0 - u)*(1.0 - 2.0 * u)*f0 + 4.0 * u*(1.0 - u)*f1 + u*(2.0 * u - 1.0)*f2;
				double b = (1.0 - u)*(1.0 - 2.0 * u)*f3 + 4.0 * u*(1.0 - u)*f4 + u*(2.0 * u - 1.0)*f5;
				double c = (1.0 - u)*(1.0 - 2.0 * u)*f6 + 4.0 * u*(1.0 - u)*f7 + u*(2.0 * u - 1.0)*f8;


				// get pixel from input image at pixel coordinate mappedIndex
				double pixel_intensity =
					(1.0 - v)*(1.0 - 2.0 * v)*a + 4.0 * v*(1.0 - v)*b + v*(2.0 * v - 1.0)*c;


				out_img->setFromNormalizedDVec4(
					size2_t(i, j),
					dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
			}
		}
	}

	void ImageUpsampler::barycentricInterpolation(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto sampleSize = samplerSize_.get();
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		ivec2 clamper = inputSize - size_t(1);

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {
				// TODO: Task 6: Updated this code to use barycentric interpolation

				float x = static_cast<float>(i) / (static_cast<float>(outputSize.x) - 1.f);
				float y = static_cast<float>(j) / (static_cast<float>(outputSize.y) - 1.f);

				x *= static_cast<float>(inputSize.x) - 1.f;
				y *= static_cast<float>(inputSize.y) - 1.f;

				glm::vec2 p(x, y);

				ivec2 A(floor(x), floor(y));
				ivec2 B = A + ivec2(1, 0);
				ivec2 C = A + ivec2(0, 1);
				ivec2 D = A + ivec2(1, 1);

				auto dist1 = glm::distance2(p, glm::vec2(A));
				auto dist2 = glm::distance2(p, glm::vec2(D));

				double u = x - A.x;
				double v = y - A.y;

				float alpha, beta, gamma;

				if (dist1 > dist2) {
					// right triangle
					std::swap(A, D);
					alpha = u + v - 1.0;
					beta = 1 - v;

					gamma = 1 - u;
				}
				else {
					std::swap(B, C);
					// left triangle
					alpha = 1 - u - v;
					beta = v;
					gamma = u;
				}

				A = glm::clamp(A, ivec2(0), clamper);
				B = glm::clamp(B, ivec2(0), clamper);
				C = glm::clamp(C, ivec2(0), clamper);

				auto fA = in_img->getAsNormalizedDouble(A);
				auto fB = in_img->getAsNormalizedDouble(B);
				auto fC = in_img->getAsNormalizedDouble(C);

				auto pixel_intensity = alpha * fA + beta * fB + gamma * fC;

				out_img->setFromNormalizedDVec4(
					size2_t(i, j),
					dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image

			}
		}
	}

	void ImageUpsampler::applyColorMap(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto samplesize = samplerSize_;
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {
				auto pixel_intensity =
					in_img->getAsNormalizedDouble(size2_t(i, j));  // get from input image
																   // scale factor for debugging
				pixel_intensity *= pixelIntensityScaleFactor_.get();

				auto scalarColor = scalarColorMapping_.sample(pixel_intensity);
				scalarColor.w = 1.0f;

				out_img->setFromNormalizedDVec4(size2_t(i, j),
					dvec4(scalarColor));  // set to output image
			}
		}
	}

}  // namespace
