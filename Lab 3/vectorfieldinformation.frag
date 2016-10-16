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
 
#include "utils/structs.glsl"

uniform sampler2D vfColor;
uniform ImageParameters vfParameters;
in vec3 texCoord_;


vec4 passThrough(vec2 coord){
    return vec4( texture2D(vfColor,coord) );
}

vec4 magnitude( vec2 coord ){
	
	vec2 p = texture2D(vfColor, coord).xy;

	
	float magnitude = sqrt(p.x*p.x + p.y*p.y);
    return vec4( magnitude);
}

vec4 divergence(vec2 coord){
	vec2 pixelSize = vfParameters.reciprocalDimensions;
	
	vec2 x_p = vec2(coord.x + pixelSize.x, coord.y);
	vec2 x_m = vec2(coord.x - pixelSize.x, coord.y);

	vec2 vx_p = texture2D(vfColor, x_p.xy).xy;
	vec2 vx_m = texture2D(vfColor, x_m.xy).xy;

	vec2 dv_dx = (vx_p - vx_m) / (2.f*pixelSize.x);

	vec2 y_p = vec2(coord.x, coord.y + pixelSize.y);
	vec2 y_m = vec2(coord.x, coord.y - pixelSize.y);

	vec2 vy_p = texture2D(vfColor, y_p.xy).xy;
	vec2 vy_m = texture2D(vfColor, y_m.xy).xy;

	vec2 dv_dy = (vy_p - vy_m) / (2.f*pixelSize.y);

    return vec4(dv_dx.x + dv_dy.y);
}

vec4 rotation(vec2 coord){
    vec2 pixelSize = vfParameters.reciprocalDimensions;

	vec2 x_p = vec2(coord.x + pixelSize.x, coord.y);
	vec2 x_m = vec2(coord.x - pixelSize.x, coord.y);

	vec2 vx_p = texture2D(vfColor, x_p.xy).xy;
	vec2 vx_m = texture2D(vfColor, x_m.xy).xy;

	vec2 dv_dx = (vx_p - vx_m) / (2.f*pixelSize.x);

	vec2 y_p = vec2(coord.x, coord.y + pixelSize.y);
	vec2 y_m = vec2(coord.x, coord.y - pixelSize.y);

	vec2 vy_p = texture2D(vfColor, y_p.xy).xy;
	vec2 vy_m = texture2D(vfColor, y_m.xy).xy;

	vec2 dv_dy = (vy_p - vy_m) / (2.f*pixelSize.y);

    return vec4(dv_dy.x - dv_dx.y);
}


void main(void) {
    FragData0 = OUTPUT(texCoord_.xy);
}
