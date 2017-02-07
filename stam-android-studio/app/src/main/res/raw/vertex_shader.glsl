attribute vec4 vPosition; 
attribute vec2 vTexCoord; 
varying vec2 v_texCoord; 

void main() { 
     v_texCoord = vTexCoord; 
	 gl_Position = vPosition; 
}