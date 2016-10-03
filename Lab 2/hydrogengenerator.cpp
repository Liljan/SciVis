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
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/tnm067lab2/processors/hydrogengenerator.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/util/volumeramutils.h>
#include <modules/base/algorithm/volume/volumeminmax.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/datastructures/volume/volumeram.h>

namespace inviwo {

    const ProcessorInfo HydrogenGenerator::processorInfo_{
        "org.inviwo.HydrogenGenerator",  // Class identifier
        "Hydrogen Generator",            // Display name
        "TNM067",                        // Category
        CodeState::Experimental,         // Code state
        Tags::None,                      // Tags
    };

    const ProcessorInfo HydrogenGenerator::getProcessorInfo() const { return processorInfo_; }

    HydrogenGenerator::HydrogenGenerator()
        : Processor()
        , volume_("volume")
        , size_("size_", "Volume Size", 16, 4, 256)
    {
        addPort(volume_);
        addProperty(size_);
    }

    void HydrogenGenerator::process() {
        auto vol = std::make_shared<Volume>(size3_t(size_), DataFloat32::get());

        auto ram = vol->getEditableRepresentation<VolumeRAM>();
        auto data = static_cast<float *>(ram->getData());
        util::IndexMapper3D index(ram->getDimensions());

        util::forEachVoxel(*ram, [&](const size3_t &pos) {
            vec3 cartesian = idTOCartesian(pos);
            data[index(pos)] = eval(cartesian);
        });

        auto minMax = util::volumeMinMax(ram);
        vol->dataMap_.dataRange = vol->dataMap_.valueRange = dvec2(minMax.first.x, minMax.second.x);

        volume_.setData(vol);
    }

    inviwo::vec3 HydrogenGenerator::cartesianToSphereical(vec3 cartesian) {

		auto pi_f = static_cast<float>(M_PI);

		auto r = sqrt(cartesian.x*cartesian.x + cartesian.y*cartesian.y + cartesian.z*cartesian.z);

		auto theta = std::acosf(cartesian.z / r);
		theta = glm::clamp(theta, 0.0f, 2.0f*pi_f);

		auto phi = std::atan2f(cartesian.y , cartesian.x);

        return vec3(r, theta, phi);
    }

    double HydrogenGenerator::eval(vec3 cartesian) {
        // Z = 1, a_0 = 1

		auto sph = cartesianToSphereical(cartesian);

		auto term_1 = 1.0 / (81.0*sqrt(6.0*M_PI));
		auto term_2 = sph.x*sph.x;
		auto term_3 = exp(-sph.x / 3.0);
		auto term_4 = 3.0*std::cos(sph.y)*std::cos(sph.y) - 1.0;

		auto wave_func =  term_1*term_2*term_3*term_4;

        return wave_func*wave_func;
    }

    inviwo::vec3 HydrogenGenerator::idTOCartesian(size3_t pos) {
        vec3 p(pos);
        p /= size_ - 1;
        return p * (36.0f) - 18.0f;
    }

} // namespace

