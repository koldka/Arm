varying float v_color;
varying float v_alpha;

void main()
{
    // Set fragment color from texture
    gl_FragColor = vec4(v_color, v_color, v_color, v_alpha);
}
