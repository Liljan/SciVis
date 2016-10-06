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

#include <modules/tnm067lab2/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>

namespace inviwo {

	size_t MarchingTetrahedra::HashFunc::max = 1;

	const ProcessorInfo MarchingTetrahedra::processorInfo_{
		"org.inviwo.MarchingTetrahedra",  // Class identifier
		"Marching Tetrahedra",            // Display name
		"TNM067",                         // Category
		CodeState::Experimental,          // Code state
		Tags::None,                       // Tags
	};
	const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

	MarchingTetrahedra::MarchingTetrahedra()
		: Processor()
		, volume_("volume")
		, mesh_("mesh")
		, isoValue_("isoValue", "ISO value", 0.1f, 0.0f, 1.0f) {
		addPort(volume_);
		addPort(mesh_);

		addProperty(isoValue_);

		volume_.onChange([&]() {
			if (!volume_.hasData()) {
				return;
			}
			NetworkLock lock(getNetwork());
			float iso = (isoValue_.get() - isoValue_.getMinValue()) /
				(isoValue_.getMaxValue() - isoValue_.getMinValue());
			auto vr = volume_.getData()->dataMap_.valueRange;
			isoValue_.setMinValue(vr.x);
			isoValue_.setMaxValue(vr.y);
			isoValue_.setIncrement(glm::abs(vr.y - vr.x) / 50.0f);
			isoValue_.set(iso * (vr.y - vr.x) + vr.x);
			isoValue_.setCurrentStateAsDefault();
		});
	}

	void MarchingTetrahedra::process() {
		auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
		MeshHelper mesh(volume_.getData());

		const auto dims = volume->getDimensions();
		MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

		float iso = isoValue_.get();

		util::IndexMapper3D index(dims);

		const static size_t tetrahedraIds[6][4] = { {0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
												   {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2} };

		size3_t pos;
		for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) {
			for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
				for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
					// Step 1: create current cell
					// Use volume->getAsDouble to query values from the volume
					// Spatial position should be between 0 and 1
					// The voxel index should be the 1D-index for the voxel
					Cell c;
					int ind{ 0 };

					for (int z = 0; z < 2; ++z) {
						for (int y = 0; y < 2; ++y) {
							for (int x = 0; x < 2; ++x) {
								c.voxels[ind].pos = glm::vec3(pos.x + x, pos.y + y, pos.z + z);
								c.voxels[ind].value = volume->getAsDouble(c.voxels[ind].pos);

								// voxel index is set to the global voxel index
								c.voxels[ind].index = index(c.voxels[ind].pos);
								++ind;
							}
						}
					}

					// Step 2: Subdivide cell into tetrahedra (hint: use tetrahedraIds)
					std::vector<Tetrahedra> tetrahedras;
					Tetrahedra t;

					for (int i = 0; i < 6; ++i) {

						t.voxels[0] = c.voxels[tetrahedraIds[i][0]];
						t.voxels[1] = c.voxels[tetrahedraIds[i][1]];
						t.voxels[2] = c.voxels[tetrahedraIds[i][2]];
						t.voxels[3] = c.voxels[tetrahedraIds[i][3]];

						tetrahedras.push_back(t);
					}

					for (const Tetrahedra& tetrahedra : tetrahedras) {
						// Step three: Calculate for tetra case index
						int caseId = 0;
						if (tetrahedra.voxels[0].value < isoValue_) caseId |= 1;
						if (tetrahedra.voxels[1].value < isoValue_) caseId |= 2;
						if (tetrahedra.voxels[2].value < isoValue_) caseId |= 4;
						if (tetrahedra.voxels[3].value < isoValue_) caseId |= 8;

						//V = V1 + (isovalue - ISO1) (V2 - V1) / (ISO2 - ISO1)
						// step four: Extract triangles
						glm::vec3 p_0 = tetrahedra.voxels[0].pos;
						glm::vec3 p_1 = tetrahedra.voxels[1].pos;
						glm::vec3 p_2 = tetrahedra.voxels[2].pos;
						glm::vec3 p_3 = tetrahedra.voxels[3].pos;
						float iso_0 = tetrahedra.voxels[0].value;
						float iso_1 = tetrahedra.voxels[1].value;
						float iso_2 = tetrahedra.voxels[2].value;
						float iso_3 = tetrahedra.voxels[3].value;

						glm::vec3 inter_0, inter_1, inter_2, inter_3;
						uint32_t ind_0, ind_1, ind_2, ind_3;

						switch (caseId)
						{
						case 0:
						case 15:
							break;
						case 1:
						case 14:
							inter_0 = p_0 + (isoValue_ - iso_0)*(p_1 - p_0) / (iso_1 - iso_0);
							inter_1 = p_0 + (isoValue_ - iso_0)*(p_3 - p_0) / (iso_3 - iso_0);
							inter_2 = p_0 + (isoValue_ - iso_0)*(p_2 - p_0) / (iso_2 - iso_0);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[0].index, tetrahedra.voxels[1].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[0].index, tetrahedra.voxels[3].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[0].index, tetrahedra.voxels[2].index);

							if (caseId == 1)
								mesh.addTriangle(ind_0, ind_1, ind_2);
							else
								mesh.addTriangle(ind_0, ind_2, ind_1);

							break;
							
						case 2:
						case 13:
							inter_0 = p_1 + (isoValue_ - iso_1)*(p_3 - p_1) / (iso_3 - iso_1);
							inter_1 = p_1 + (isoValue_ - iso_1)*(p_2 - p_1) / (iso_2 - iso_1);
							inter_2 = p_1 + (isoValue_ - iso_1)*(p_0 - p_1) / (iso_0 - iso_1);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[1].index, tetrahedra.voxels[3].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[1].index, tetrahedra.voxels[2].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[1].index, tetrahedra.voxels[0].index);

							if (caseId == 13)
								mesh.addTriangle(ind_0, ind_1, ind_2);
							else
								mesh.addTriangle(ind_0, ind_2, ind_1);
							break;
						case 3:
						case 12:
							inter_0 = p_1 + (isoValue_ - iso_1)*(p_2 - p_1) / (iso_2 - iso_1);
							inter_1 = p_1 + (isoValue_ - iso_1)*(p_3 - p_1) / (iso_3 - iso_1);
							inter_2 = p_0 + (isoValue_ - iso_0)*(p_3 - p_0) / (iso_3 - iso_0);
							inter_3 = p_0 + (isoValue_ - iso_0)*(p_2 - p_0) / (iso_2 - iso_0);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[1].index, tetrahedra.voxels[2].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[1].index, tetrahedra.voxels[3].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[0].index, tetrahedra.voxels[3].index);
							ind_3 = mesh.addVertex(inter_3, tetrahedra.voxels[0].index, tetrahedra.voxels[2].index);

							if (caseId == 3) {
								mesh.addTriangle(ind_0, ind_1, ind_2);
								mesh.addTriangle(ind_0, ind_2, ind_3);
							}
							else {
								mesh.addTriangle(ind_2, ind_1, ind_0);
								mesh.addTriangle(ind_3, ind_2, ind_0);
							}							

							break;
						case 4:
						case 11:
							inter_0 = p_2 + (isoValue_ - iso_2)*(p_0 - p_2) / (iso_0 - iso_2);
							inter_1 = p_2 + (isoValue_ - iso_2)*(p_1 - p_2) / (iso_1 - iso_2);
							inter_2 = p_2 + (isoValue_ - iso_2)*(p_3 - p_2) / (iso_3 - iso_2);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[2].index, tetrahedra.voxels[0].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[2].index, tetrahedra.voxels[1].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[2].index, tetrahedra.voxels[3].index);

							if (caseId == 11)
								mesh.addTriangle(ind_0, ind_1, ind_2);
							else
								mesh.addTriangle(ind_0, ind_2, ind_1);

							break;
						case 5:
						case 10:
							inter_0 = p_0 + (isoValue_ - iso_0)*(p_3 - p_0) / (iso_3 - iso_0);
							inter_1 = p_0 + (isoValue_ - iso_0)*(p_1 - p_0) / (iso_1 - iso_0);
							inter_2 = p_2 + (isoValue_ - iso_2)*(p_1 - p_2) / (iso_1 - iso_2);
							inter_3 = p_2 + (isoValue_ - iso_2)*(p_3 - p_2) / (iso_3 - iso_2);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[0].index, tetrahedra.voxels[3].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[0].index, tetrahedra.voxels[1].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[2].index, tetrahedra.voxels[1].index);
							ind_3 = mesh.addVertex(inter_3, tetrahedra.voxels[2].index, tetrahedra.voxels[3].index);

							if (caseId == 10) {
								mesh.addTriangle(ind_0, ind_1, ind_2);
								mesh.addTriangle(ind_0, ind_2, ind_3);
							}
							else {
								mesh.addTriangle(ind_2, ind_1, ind_0);
								mesh.addTriangle(ind_3, ind_2, ind_0);
							}
							break;
						case 6:
						case 9:
							inter_0 = p_1 + (isoValue_ - iso_1)*(p_0 - p_1) / (iso_0 - iso_1);
							inter_1 = p_1 + (isoValue_ - iso_1)*(p_3 - p_1) / (iso_3 - iso_1);
							inter_2 = p_2 + (isoValue_ - iso_2)*(p_3 - p_2) / (iso_3 - iso_2);
							inter_3 = p_2 + (isoValue_ - iso_2)*(p_0 - p_2) / (iso_0 - iso_2);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[1].index, tetrahedra.voxels[0].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[1].index, tetrahedra.voxels[3].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[2].index, tetrahedra.voxels[3].index);
							ind_3 = mesh.addVertex(inter_3, tetrahedra.voxels[2].index, tetrahedra.voxels[0].index);

							if (caseId == 9) {
								mesh.addTriangle(ind_0, ind_1, ind_3);
								mesh.addTriangle(ind_1, ind_2, ind_3);
							}
							else {
								mesh.addTriangle(ind_3, ind_1, ind_0);
								mesh.addTriangle(ind_3, ind_2, ind_1);
							}
							break;
						case 7:
						case 8:
							inter_0 = p_3 + (isoValue_ - iso_3)*(p_2 - p_3) / (iso_2 - iso_3);
							inter_1 = p_3 + (isoValue_ - iso_3)*(p_0 - p_3) / (iso_0 - iso_3);
							inter_2 = p_3 + (isoValue_ - iso_3)*(p_1 - p_3) / (iso_1 - iso_3);

							ind_0 = mesh.addVertex(inter_0, tetrahedra.voxels[3].index, tetrahedra.voxels[2].index);
							ind_1 = mesh.addVertex(inter_1, tetrahedra.voxels[3].index, tetrahedra.voxels[0].index);
							ind_2 = mesh.addVertex(inter_2, tetrahedra.voxels[3].index, tetrahedra.voxels[1].index);

							if (caseId == 8)
								mesh.addTriangle(ind_0, ind_1, ind_2);
							else
								mesh.addTriangle(ind_0, ind_2, ind_1);
							break;
						}
					}
				}
			}
		}

		mesh_.setData(mesh.toBasicMesh());
	}

	MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
		: edgeToVertex_()
		, vertices_()
		, mesh_(std::make_shared<BasicMesh>())
		, indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
		mesh_->setModelMatrix(vol->getModelMatrix());
		mesh_->setWorldMatrix(vol->getWorldMatrix());
	}

	void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
		ivwAssert(i0 != i1, "i0 and i1 should not be the same value");
		ivwAssert(i0 != i2, "i0 and i2 should not be the same value");
		ivwAssert(i1 != i2, "i1 and i2 should not be the same value");

		indexBuffer_->add(static_cast<glm::uint32_t>(i0));
		indexBuffer_->add(static_cast<glm::uint32_t>(i1));
		indexBuffer_->add(static_cast<glm::uint32_t>(i2));

		auto a = vertices_[i0].pos;
		auto b = vertices_[i1].pos;
		auto c = vertices_[i2].pos;

		vec3 n = glm::normalize(glm::cross(b - a, c - a));
		vertices_[i0].normal += n;
		vertices_[i1].normal += n;
		vertices_[i2].normal += n;
	}

	std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
		for (auto& vertex : vertices_) {
			vertex.normal = glm::normalize(vertex.normal);
		}
		mesh_->addVertices(vertices_);
		return mesh_;
	}

	std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
		ivwAssert(i != j, "i and j should not be the same value");
		if (j < i) {
			return addVertex(pos, j, i);
		}

		auto edge = std::make_pair(i, j);

		auto it = edgeToVertex_.find(edge);

		if (it == edgeToVertex_.end()) {
			edgeToVertex_[edge] = vertices_.size();
			vertices_.push_back({ pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f) });
			return vertices_.size() - 1;
		}

		return it->second;
	}

}  // namespace
