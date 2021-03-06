#include "gl_program.h"
#include <assert.h>
#include <errno.h>

#include <sstream>
#include <stdexcept>
#include <cstring>
#include <chrono>

GLProgram::GLProgram(const char *common, const char *vertex, const char *fragment, bool is_file) {
	GLint status;
	m_common = common;
	m_program_id = glCreateProgram();

	m_vertex_id = LoadShader(GL_VERTEX_SHADER, vertex, is_file);
	m_fragment_id = LoadShader(GL_FRAGMENT_SHADER, fragment, is_file);
	glAttachShader(m_program_id, m_vertex_id);
	glAttachShader(m_program_id, m_fragment_id);

	glLinkProgram(m_program_id);
	glGetProgramiv(m_program_id, GL_LINK_STATUS, &status);
	if (!status) {
		GLint msg_len;
		char *msg;
		std::stringstream s;

		glGetProgramiv(m_program_id, GL_INFO_LOG_LENGTH, &msg_len);
		msg = new char[msg_len];
		glGetProgramInfoLog(m_program_id, msg_len, NULL, msg);

		s << "Failed to link shaders: " << msg;
		delete[] msg;
		throw std::invalid_argument(s.str());
	}
}

GLProgram::~GLProgram() {
	glDeleteShader(m_fragment_id);
	glDeleteShader(m_vertex_id);
	glDeleteProgram(m_program_id);
}

GLuint GLProgram::GetId() {
	return m_program_id;
}

GLuint GLProgram::LoadShader(GLenum shader_type, const char *source, bool is_file) {
	GLint status;
	GLuint shader_id;
	char *shader_source = NULL;
	if (is_file) {
		shader_source = ReadFile(source);
	} else {
		size_t length = strlen(source);
		size_t common_length = m_common ? strlen(m_common) : 0;
		shader_source = new char[common_length + length + 1];
		if (common_length) {
			strncpy(shader_source, m_common, common_length);
		}
		if (length) {
			strncpy(shader_source + common_length, source, length);
		}
		shader_source[common_length + length] = '\0';
	}

	if (!shader_source) {
		std::stringstream s;
		const char *error = std::strerror(errno);
		s << "Could not load " << source << ": " << error;
		throw std::invalid_argument(s.str());
	}

	shader_id = glCreateShader(shader_type);
	glShaderSource(shader_id, 1, (const GLchar**) &shader_source, NULL);
	glCompileShader(shader_id);

	glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);
	if (!status) {
		GLint msg_len;
		char *msg;
		std::stringstream s;

		glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &msg_len);
		msg = new char[msg_len];
		glGetShaderInfoLog(shader_id, msg_len, NULL, msg);

		s << "Failed to compile " << shader_source << ": " << msg;
		delete[] msg;
		throw std::invalid_argument(s.str());
	}
	delete[] shader_source;

	return shader_id;
}

char* GLProgram::ReadFile(const char *file) {
	std::FILE *fp = std::fopen(file, "rb");
	char *ret = NULL;
	size_t length;
	size_t common_length = m_common ? strlen(m_common) : 0;

	if (fp) {
		std::fseek(fp, 0, SEEK_END);
		length = std::ftell(fp);
		std::fseek(fp, 0, SEEK_SET);

		ret = new char[common_length + length + 1];
		if (common_length) {
			strncpy(ret, m_common, common_length);
		}
		length = std::fread(ret + common_length, 1, length, fp);
		ret[common_length + length] = '\0';

		std::fclose(fp);
	}

	return ret;
}

void *GLProgram_new(const char *common, const char *vertex_file, const char *fragment_file, bool is_file) {
	return (void*) new GLProgram(common, vertex_file, fragment_file, is_file);
}
GLuint GLProgram_GetId(const void *_this) {
	return ((GLProgram*) _this)->GetId();
}
void GLProgram_delete(const void *_this) {
	delete ((GLProgram*) _this);
}
