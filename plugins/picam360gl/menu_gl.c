


void init_menu(uint32_t font_size) {
#ifdef USE_GLES
	// all the shaders have at least texture unit 0 active so
	// activate it now and leave it active
	glActiveTexture(GL_TEXTURE0);

	/* Texture atlas to store individual glyphs */
	lg_freetypegles.atlas = texture_atlas_new(1024, 1024, 1);
#ifdef USE_GLES
	lg_freetypegles.font = texture_font_new(lg_freetypegles.atlas, "./libs/freetypeGlesRpi/fonts/custom.ttf", font_size);
#endif
	/* Cache some glyphs to speed things up */
	texture_font_load_glyphs(lg_freetypegles.font, L" !\"#$%&'()*+,-./0123456789:;<=>?"
			L"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_"
			L"`abcdefghijklmnopqrstuvwxyz{|}~");

	{
		int ret;
		const char *fsh_filepath = "/tmp/tmp.fsh";
		const char *vsh_filepath = "/tmp/tmp.vsh";
		int fsh_fd = open(fsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		int vsh_fd = open(vsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		ret = write(fsh_fd, freetype_fsh, freetype_fsh_len);
		ret = write(vsh_fd, freetype_vsh, freetype_vsh_len);
		close(fsh_fd);
		close(vsh_fd);
		lg_freetypegles.model.program = GLProgram_new("", vsh_filepath, fsh_filepath, true);
		remove(fsh_filepath);
		remove(vsh_filepath);
	}
#ifdef USE_GLES
	texture_atlas_upload(lg_freetypegles.atlas);
#endif
#endif
}
void deinit_menu() {
	//todo
}



void menu_redraw(MENU_T *root, char *_status, uint32_t screen_width, uint32_t screen_height, uint32_t frame_width, uint32_t frame_height, bool stereo) {

	int program = GLProgram_GetId(lg_freetypegles.model.program);
	glUseProgram(program);

	vector_t * vVector = vector_new(sizeof(GLfloat));
	int line = 0;

	if (_status) {
		const int MAX_STATUS_LEN = 1024;
		wchar_t status[1024];
		swprintf(status, MAX_STATUS_LEN - 1, L"%s", _status);
		status[MAX_STATUS_LEN - 1] = L'\0';	//fail safe

		vec2 pen = { };
		vec4 color = { 1, 1, 1, 1 };
		vec4 back_color = { 0.2, 0.2, 0.2, 1 };

		wchar_t *ptr;
		wchar_t *tok = wcstok(status, L"\n", &ptr);
		while (tok) {
			pen.x = -((float) screen_width / 2 - lg_freetypegles.font->size / 8);
			pen.y = ((float) screen_height / 2 - lg_freetypegles.font->size / 8) - lg_freetypegles.font->size * (line + 1);
			add_text(vVector, lg_freetypegles.font, tok, &back_color, &pen);

			pen.x = -((float) screen_width / 2);
			pen.y = ((float) screen_height / 2) - lg_freetypegles.font->size * (line + 1);
			add_text(vVector, lg_freetypegles.font, tok, &color, &pen);

			line++;
			tok = wcstok(NULL, L"\n", &ptr);
		}
	}
	if (root && root->activated) {
		expand_menu(root, vVector, &line, screen_width, screen_height, 0);
	}

	// Use the program object
	glUseProgram(program);

	int vertexHandle, texHandle, samplerHandle, colorHandle, mvpHandle;
	// Bind vPosition to attribute 0
	vertexHandle = glGetAttribLocation(program, "a_position");
	texHandle = glGetAttribLocation(program, "a_st");
	colorHandle = glGetAttribLocation(program, "a_color");
	samplerHandle = glGetUniformLocation(program, "texture_uniform");

	mvpHandle = glGetUniformLocation(program, "u_mvp");

	float a = 1.0f / (screen_width / 2);
	float b = 1.0f / (screen_height / 2);

	GLfloat mvp[] = { //
			//
					a, 0, 0, 0,		//
					0, b, 0, 0,		//
					0, 0, 1.0, 0,		//
					0, 0, 0, 1.0		//
			};
	glUniformMatrix4fv(mvpHandle, 1, GL_FALSE, (GLfloat *) mvp);

// Load the vertex data
	glVertexAttribPointer(vertexHandle, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), vVector->items);
	glEnableVertexAttribArray(vertexHandle);
	glVertexAttribPointer(texHandle, 2, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), (GLfloat*) vVector->items + 3);
	glEnableVertexAttribArray(texHandle);
	glVertexAttribPointer(colorHandle, 4, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), (GLfloat*) vVector->items + 5);
	glEnableVertexAttribArray(colorHandle);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, lg_freetypegles.atlas->id);

	glUniform1i(samplerHandle, 0);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	if (stereo) {
		int offset_x = (screen_width - frame_width) / 2;
		int offset_y = (screen_height - frame_height) / 2;
		for (int i = 0; i < 2; i++) {
			glViewport(offset_x + i * screen_width, offset_y, (GLsizei) frame_width, (GLsizei) frame_height);
			glDrawArrays(GL_TRIANGLES, 0, vVector->size / 9);
		}
	} else {
		int offset_x = (screen_width - frame_width) / 2;
		int offset_y = (screen_height - frame_height) / 2;
		glViewport(offset_x, offset_y, (GLsizei) frame_width, (GLsizei) frame_height);
		glDrawArrays(GL_TRIANGLES, 0, vVector->size / 9);
	}

	glDisableVertexAttribArray(vertexHandle);
	glDisableVertexAttribArray(texHandle);
	glDisableVertexAttribArray(colorHandle);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);

	vector_delete(vVector);
}
