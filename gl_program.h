
#ifndef _GLPROGRAM_H
#define _GLPROGRAM_H
#ifdef USE_GLES
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#else
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
//#include "GL/gl.h"
//#include "GL/glut.h"
//#include "GL/glext.h"
#endif

#ifdef __cplusplus

class GLProgram {
public:
	GLProgram(const char *vertex_file, const char *fragment_file);
	virtual ~GLProgram();

	GLuint GetId();
	operator GLuint() {
		return m_program_id;
	}
	;
private:
	GLuint m_vertex_id, m_fragment_id, m_program_id;

	GLuint LoadShader(GLenum shader_type, const char *source_file);
	char* ReadFile(const char *file);
};

extern "C" {

#endif

void *GLProgram_new(const char *vertex_file, const char *fragment_file);
GLuint GLProgram_GetId(const void *_this);
void GLProgram_delete(const void *_this);

#ifdef __cplusplus
}
#endif

#endif
