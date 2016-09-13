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
				// Divide the output indices with a scaling factor
				size2_t mappedIndex(i / sampleSize, j / sampleSize);
				mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

				// get pixel from input image at pixel coordinate mappedIndex
				auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

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
				// TODO: Task 4: Updated this code to use bilinear interpolation
				size2_t mappedIndex(i / j);
				mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

				size2_t A(i / sampleSize, j / sampleSize);
				size2_t B(i / sampleSize + 1, j / sampleSize);
				size2_t C(i / sampleSize, j / sampleSize + 1);
				size2_t D(i / sampleSize + 1, j / sampleSize + 1);

				double intesityAB, intesityCD;
				if (A.x != B.x) {
					intesityAB =
						(B.x*sampleSize - mappedIndex.x) / (B.x - A.x) * in_img->getAsNormalizedDouble(A) +
						(mappedIndex.x - A.x*sampleSize) / (B.x - A.x) * in_img->getAsNormalizedDouble(B);

					intesityCD =
						(D.x*sampleSize - mappedIndex.x) / (D.x - C.x) * in_img->getAsNormalizedDouble(C) +
						(mappedIndex.x - C.x*sampleSize) / (D.x - C.x) * in_img->getAsNormalizedDouble(D);
				} else {
					intesityAB = in_img->getAsNormalizedDouble(A);
					intesityCD = in_img->getAsNormalizedDouble(C);
				}

				double pixel_intensity;
				if (A.y == C.y && A.x == B.x) {
					pixel_intensity = in_img->getAsNormalizedDouble(A);
				}
				else {
					pixel_intensity =
						(A.y*sampleSize - mappedIndex.y) / (A.y - C.y) * intesityAB +
						(mappedIndex.y - C.y*sampleSize) / (A.y - C.y) * intesityCD;
				}

				out_img->setFromNormalizedDVec4(
					size2_t(i, j),
					dvec4(pixel_intensity * pixelIntensityScaleFactor_.get()));  // set to output image
			}
		}
	}

	void ImageUpsampler::quadraticInterpolation(const Image *inputImage, Image *outputImage) {
		auto out_img = outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>();
		auto in_img = inputImage->getColorLayer()->getRepresentation<LayerRAM>();

		auto sampleSize = samplerSize_.get();
		auto inputSize = inputImage->getDimensions();
		auto outputSize = out_img->getDimensions();

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {
				// TODO: Task 5: Updated this code to use quadratic interpolation
				size2_t mappedIndex(i, j);
				mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

				// get pixel from input image at pixel coordinate mappedIndex
				auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

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

		for (size_t i = 0; i < outputSize.x; i++) {
			for (size_t j = 0; j < outputSize.y; j++) {
				// TODO: Task 6: Updated this code to use barycentric interpolation
				size2_t mappedIndex(i, j);
				mappedIndex = glm::clamp(mappedIndex, size2_t(0), inputSize - size_t(1));

				// get pixel from input image at pixel coordinate mappedIndex
				auto pixel_intensity = in_img->getAsNormalizedDouble(mappedIndex);

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
				double x = static_cast<double>(i) / outputSize.x;
				double y = static_cast<double>(j) / outputSize.y;

				size2_t mappedIndex(x * (inputSize.x - 1), y * (inputSize.y - 1));
				auto pixel_intensity =
					in_img->getAsNormalizedDouble(mappedIndex);  // get from input image

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
