
#include "gl_program.h"

GLProgram::GLProgram(const char *vertex_file, const char *fragment_file) {
	GLint status;
	m_program_id = glCreateProgram();

	m_vertex_id = LoadShader(GL_VERTEX_SHADER, vertex_file);
	m_fragment_id = LoadShader(GL_FRAGMENT_SHADER, fragment_file);
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

GLuint GLProgram::LoadShader(GLenum shader_type, const char *source_file) {
	GLint status;
	GLuint shader_id;
	char *shader_source = ReadFile(source_file);

	if (!shader_source) {
		std::stringstream s;
		const char *error = std::strerror(errno);
		s << "Could not load " << source_file << ": " << error;
		throw std::invalid_argument(s.str());
	}

	shader_id = glCreateShader(shader_type);
	glShaderSource(shader_id, 1, (const GLchar**) &shader_source, NULL);
	glCompileShader(shader_id);
	delete[] shader_source;

	glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);
	if (!status) {
		GLint msg_len;
		char *msg;
		std::stringstream s;

		glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &msg_len);
		msg = new char[msg_len];
		glGetShaderInfoLog(shader_id, msg_len, NULL, msg);

		s << "Failed to compile " << source_file << ": " << msg;
		delete[] msg;
		throw std::invalid_argument(s.str());
	}

	return shader_id;
}

char* GLProgram::ReadFile(const char *file) {
	std::FILE *fp = std::fopen(file, "rb");
	char *ret = NULL;
	size_t length;

	if (fp) {
		std::fseek(fp, 0, SEEK_END);
		length = std::ftell(fp);
		std::fseek(fp, 0, SEEK_SET);

		ret = new char[length + 1];
		length = std::fread(ret, 1, length, fp);
		ret[length] = '\0';

		std::fclose(fp);
	}

	return ret;
}

void *GLProgram_new(const char *vertex_file, const char *fragment_file)
{
	return (void*)new GLProgram(vertex_file, fragment_file);
}
GLuint GLProgram_GetId(const void *_this)
{
	return ((GLProgram*)_this)->GetId();
}
void GLProgram_delete(const void *_this)
{
	delete ((GLProgram*)_this);
}
