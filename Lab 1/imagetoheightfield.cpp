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

#include <modules/tnm067lab1/processors/imagetoheightfield.h>
#include <modules/tnm067lab1/utils/scalartocolormapping.h>
#include <inviwo/core/util/imageramutils.h>


namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo ImageToHeightfield::processorInfo_{
    "org.inviwo.ImageToHeightfield",  // Class identifier
    "ImageToHeightfield",             // Display name
    "TNM067",                         // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};


const ProcessorInfo ImageToHeightfield::getProcessorInfo() const { return processorInfo_; }

ImageToHeightfield::ImageToHeightfield()
    : Processor()
    , imageInport_("imageInport")
    , meshOutport_("meshOutport")
    , heightScaleFactor_("heightScaleFactor", "Height Scale Factor", 1.0f, 0.001, 2.0f, 0.001f) 
{
    addPort(imageInport_);
    addPort(meshOutport_);
    addProperty(heightScaleFactor_);
}

void ImageToHeightfield::process() {
    buildMesh();
    meshOutport_.setData(mesh_);
}

void ImageToHeightfield::buildMesh() {
    auto inImage = imageInport_.getData();
    auto dims = inImage->getDimensions();

    mesh_ = std::make_shared<BasicMesh>();

    auto ib = mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None);
    std::vector<BasicMesh::Vertex> vertices;
    auto ibVector = ib->getDataContainer();
    
    auto bufferSize = 24 * dims.x * dims.y;
    ibVector->reserve(bufferSize);
    vertices.reserve(bufferSize);


    vec2 cellSize = 1.0f / vec2(dims);
    auto img = inImage->getColorLayer()->getRepresentation<LayerRAM>();
    util::forEachPixel(*img, [&](const size2_t &pos) {
        unsigned int startID = static_cast<unsigned int>( vertices.size());

        vec2 origin2D = vec2(pos)*cellSize;
        vec3 origin(origin2D.x,0.0f, origin2D.y);
        
		// Get the image value of the current pixel
		double height = img->getAsDouble(pos);
		// Get the color
        vec4 color = scalarColorMapping_.sample(height);
		// Scale the height
		height *= heightScaleFactor_;

        /****************************************
        BOTTOM
        *****************************************/
        vertices.push_back({ origin + vec3(0,0,0) , vec3(0,-1,0) , origin + vec3(0,0,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,0,0) , vec3(0,-1,0) , origin + vec3(cellSize.x,0,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,0,cellSize.y) , vec3(0,-1,0) , origin + vec3(cellSize.x,0,cellSize.y) , color });
        vertices.push_back({ origin + vec3(0,0,cellSize.y) , vec3(0,-1,0) , origin + vec3(0,0,cellSize.y) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });



        /****************************************
        TOP
        *****************************************/
        startID = static_cast<unsigned int>(vertices.size());
        vertices.push_back({ origin + vec3(0,height,0) , vec3(0,1,0) , origin + vec3(0,height,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,0) , vec3(0,1,0) , origin + vec3(cellSize.x,height,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,cellSize.y) , vec3(0,1,0) , origin + vec3(cellSize.x,height,cellSize.y) , color });
        vertices.push_back({ origin + vec3(0,height,cellSize.y) , vec3(0,1,0) , origin + vec3(0,height,cellSize.y) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });




        /****************************************
        LEFT
        *****************************************/
        startID = static_cast<unsigned int>(vertices.size());
        vertices.push_back({ origin + vec3(0,0,0) , vec3(-1,0,0) , origin + vec3(0,0,0) , color });
        vertices.push_back({ origin + vec3(0,0,cellSize.y) , vec3(-1,0,0) , origin + vec3(0,0,cellSize.y) , color });
        vertices.push_back({ origin + vec3(0,height,cellSize.y) , vec3(-1,0,0) , origin + vec3(0,height,cellSize.y) , color });
        vertices.push_back({ origin + vec3(0,height,0) , vec3(-1,0,0) , origin + vec3(0,height,0) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });


        /****************************************
        RIGHT
        *****************************************/
        startID = static_cast<unsigned int>(vertices.size());
        vertices.push_back({ origin + vec3(cellSize.x,0,0)          , vec3(1,0,0) , origin + vec3(cellSize.x,0,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,0,cellSize.y) , vec3(1,0,0) , origin + vec3(cellSize.x,0,cellSize.y) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,cellSize.y) , vec3(1,0,0) , origin + vec3(cellSize.x,height,cellSize.y) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,0) , vec3(1,0,0) , origin + vec3(cellSize.x,height,0) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });




        /****************************************
        FRONT
        *****************************************/
        startID = static_cast<unsigned int>(vertices.size());
        vertices.push_back({ origin + vec3(0,0,0) , vec3(0,0,-1) , origin + vec3(0,0,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,0,0) , vec3(0,0,-1) , origin + vec3(cellSize.x,0,0) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,0) , vec3(0,0,-1) , origin + vec3(cellSize.x,height,0) , color });
        vertices.push_back({ origin + vec3(0,height,0) , vec3(0,0,-1) , origin + vec3(0,height,0) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });





        /****************************************
        BACK
        *****************************************/
        startID = static_cast<unsigned int>(vertices.size());
        vertices.push_back({ origin + vec3(0,0,cellSize.y) , vec3(0,0,1) , origin + vec3(0,0,cellSize.y) , color });
        vertices.push_back({ origin + vec3(cellSize.x,0,cellSize.y) , vec3(0,0,1) , origin + vec3(cellSize.x,0,cellSize.y) , color });
        vertices.push_back({ origin + vec3(cellSize.x,height,cellSize.y) , vec3(0,0,1) , origin + vec3(cellSize.x,height,cellSize.y) , color });
        vertices.push_back({ origin + vec3(0,height,cellSize.y) , vec3(0,0,1) , origin + vec3(0,height,cellSize.y) , color });

        ib->add({
            startID + 0,
            startID + 1,
            startID + 2,

            startID + 0,
            startID + 2,
            startID + 3
        });

        
    });


    mesh_->addVertices(vertices);

}

}  // namespace

