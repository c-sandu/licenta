#version 330

// TODO: get color value from vertex shader
in vec3 world_position;
in vec3 world_normal;

// Uniforms for light properties
uniform vec3 light_direction;
uniform vec3 light_position;
uniform vec3 eye_position;

uniform float material_kd;
uniform float material_ks;
uniform int material_shininess;

uniform vec3 object_color;

layout(location = 0) out vec4 out_color;

void main()
{
	vec3 L = normalize(light_position - world_position);
	vec3 V = normalize(eye_position - world_position);
	vec3 H = normalize(L + V);
	vec3 N = world_normal;

	// define ambient light component
	float ambient_light = 0.25;

	// compute diffuse light component
	float diffuse_light = material_kd * max(dot(N, L), 0);

	// compute specular light component
	float specular_light = 0;

	if (diffuse_light > 0)
	{
		specular_light = material_ks * pow(max(dot(N, H), 0), material_shininess);
	}

	float attenuation = 1.0 / pow(distance(light_position, world_position), 2);

	// compute light
	vec3 color = (ambient_light + 1 * (diffuse_light + specular_light)) * object_color;

	// write pixel out color
	out_color = vec4(color, 1.0);
}