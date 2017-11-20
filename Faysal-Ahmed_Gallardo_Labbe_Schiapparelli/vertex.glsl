uniform mat4 mvp_matrix;

attribute vec4 a_position;
attribute float a_color;
attribute float a_alpha;

varying float v_color;
varying float v_alpha;

void main()
{
    // Calculate vertex position in screen space
    gl_Position = mvp_matrix * a_position;

    v_color = a_color;
    v_alpha = a_alpha;
}
