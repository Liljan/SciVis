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

#ifndef GLSL_VERSION_150
#extension GL_EXT_gpu_shader4 : enable
#extension GL_EXT_geometry_shader4 : enable
#endif

layout(points) in;
layout(triangle_strip, max_vertices = 96) out;

uniform float radius;

in mat2 vJ[1];

out vec3 color;

const float pi = 3.14159265359;
const float twoPi = 6.28318530718;

void main(void) {
#ifndef GLSL_VERSION_150
    vec2 center = gl_PositionIn[0].xy;
#else
    vec2 center = gl_in[0].gl_Position.xy;
#endif

    mat2 J = vJ[0];

	mat2 Jsym = 0.5 * (J + transpose(J));

	//Jsym = mat2(30);

    for (int i = 0; i < 32; i++) {
		
		if(i < 4 || i >= 28)
			color = vec3(0.106, 0.620, 0.467);
		else if(i <= 12)
			color = vec3(0.851, 0.373, 0.008);
		else if(i <= 20)
			color = vec3(0.459, 0.439, 0.702);
		else	
			color = vec3(0.906, 0.061, 0.541);

        float a1 = twoPi * (i / 32.0);        // angle from y-axis of second point in the triangle
        float a2 = twoPi * ((i + 1) / 32.0);  // angle from y-axis of third point in the triangle

        gl_Position = vec4(center, 0, 1);  // first point in triangle is in the origin of the circle
        EmitVertex();

		// calculate an offset vector for the second point in the triangle based on a1
        vec2 o1 = vec2(cos(a1), sin(a1)) * Jsym;		

        gl_Position = vec4(center + o1 * radius, 0, 1);
        EmitVertex();

		 // calculate an offset vector for the second point in the triangle based on a2
        vec2 o2 = vec2(cos(a2), sin(a2)) * Jsym; 

        gl_Position = vec4(center + o2 * radius, 0, 1);
        EmitVertex();

        EndPrimitive();
    }
}
