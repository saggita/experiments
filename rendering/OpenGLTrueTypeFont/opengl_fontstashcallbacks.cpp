
#include "fontstash.h"
#include "../rendertest/OpenGLInclude.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


static unsigned int s_indexData[INDEX_COUNT];
GLuint s_indexArrayObject, s_indexBuffer;
GLuint s_vertexArrayObject,s_vertexBuffer;

extern   GLuint m_shaderProg;
extern    GLint m_positionUniform;
extern    GLint m_colourAttribute;
extern    GLint m_positionAttribute;
extern    GLint m_textureAttribute;
extern    GLuint m_vertexBuffer;
extern    GLuint m_vertexArrayObject;
extern    GLuint  m_indexBuffer;
extern    GLuint m_texturehandle;




void display2() {
    
    GLint err = glGetError();
    assert(err==GL_NO_ERROR);
   // glViewport(0,0,10,10);
    
	const float timeScale = 0.008f;
	
    glUseProgram(m_shaderProg);
    glBindBuffer(GL_ARRAY_BUFFER, s_vertexBuffer);
    glBindVertexArray(s_vertexArrayObject);
    
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    
    //   glBindTexture(GL_TEXTURE_2D,m_texturehandle);
    
    
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    vec2 p( 0.f,0.f);//?b?0.5f * sinf(timeValue), 0.5f * cosf(timeValue) );
    glUniform2fv(m_positionUniform, 1, (const GLfloat *)&p);
    
    err = glGetError();
    assert(err==GL_NO_ERROR);
	err = glGetError();
    assert(err==GL_NO_ERROR);
    
    glEnableVertexAttribArray(m_positionAttribute);
	err = glGetError();
    assert(err==GL_NO_ERROR);
    
    glEnableVertexAttribArray(m_colourAttribute);
	err = glGetError();
    assert(err==GL_NO_ERROR);
    
	glEnableVertexAttribArray(m_textureAttribute);
    
    glVertexAttribPointer(m_positionAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)0);
    glVertexAttribPointer(m_colourAttribute  , 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)sizeof(vec4));
    glVertexAttribPointer(m_textureAttribute , 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)(sizeof(vec4)+sizeof(vec4)));
	err = glGetError();
    assert(err==GL_NO_ERROR);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
    
    //glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    int indexCount = 6;
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    //	glutSwapBuffers();
}



void OpenGL2UpdateTextureCallback(sth_texture* texture, sth_glyph* glyph, int textureWidth, int textureHeight)
{

	if (glyph)
	{
		// Update texture (entire texture, could use glyph to update partial texture using glTexSubImage2D)
		GLuint* gltexture = (GLuint*) texture->m_userData;
		
		glBindTexture(GL_TEXTURE_2D, *gltexture);
		glPixelStorei(GL_UNPACK_ALIGNMENT,1);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, textureWidth, textureHeight, 0, GL_RED, GL_UNSIGNED_BYTE, texture->m_texels);
	
		GLenum err = glGetError();
	    assert(err==GL_NO_ERROR);

	} else
	{
		if (textureWidth && textureHeight)
		{
			GLuint* texId = new GLuint;
			texture->m_userData = texId;


			//create new texture
			glGenTextures(1, texId);
			GLenum err = glGetError();
			assert(err==GL_NO_ERROR);

			
	
			glBindTexture(GL_TEXTURE_2D, *texId);
			texture->m_texels = (unsigned char*)malloc(textureWidth*textureHeight);
			memset(texture->m_texels,0,textureWidth*textureHeight);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, textureWidth, textureHeight, 0, GL_RED, GL_UNSIGNED_BYTE, texture->m_texels);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			////////////////////////////
			//create the other data
			{
				glGenVertexArrays(1, &s_vertexArrayObject);
				glBindVertexArray(s_vertexArrayObject);
        
				glGenBuffers(1, &s_vertexBuffer);
				glBindBuffer(GL_ARRAY_BUFFER, s_vertexBuffer);
				glBufferData(GL_ARRAY_BUFFER, VERT_COUNT * sizeof(Vertex), texture->newverts, GL_DYNAMIC_DRAW);
				GLuint err = glGetError();
				assert(err==GL_NO_ERROR);
        
				for (int i=0;i<INDEX_COUNT;i++)
				{
					s_indexData[i] = i;
				}
            
				glGenBuffers(1, &s_indexBuffer);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_indexBuffer);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER,INDEX_COUNT*sizeof(int), s_indexData,GL_STATIC_DRAW);
        
				err = glGetError();
				assert(err==GL_NO_ERROR);
			}
		} else
		{
			//delete texture
			if (texture->m_userData)
			{
				
				GLuint* id = (GLuint*)texture->m_userData;

				//glDeleteTextures(1, id);
				//delete id;
				//texture->m_userData = 0;
			}

		}

	}
}

void OpenGL2RenderCallback(sth_texture* texture)
{

	GLuint* texId = (GLuint*) texture->m_userData;

    GLint err;
	err = glGetError();
    assert(err==GL_NO_ERROR);
            
    glActiveTexture(GL_TEXTURE0);
    err = glGetError();
    assert(err==GL_NO_ERROR);
            
    glBindTexture(GL_TEXTURE_2D, *texId);
    err = glGetError();
    assert(err==GL_NO_ERROR);
    // glBindBuffer(GL_ARRAY_BUFFER, s_vertexBuffer);
    // glBindVertexArray(s_vertexArrayObject);
    glBufferData(GL_ARRAY_BUFFER, texture->nverts * sizeof(Vertex), &texture->newverts[0].position.p[0], GL_DYNAMIC_DRAW);
    err = glGetError();
    assert(err==GL_NO_ERROR);
            
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_indexBuffer);
            
    //glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    int indexCount = texture->nverts;
    err = glGetError();
    assert(err==GL_NO_ERROR);
            
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    err = glGetError();
    assert(err==GL_NO_ERROR);
		


}


void dumpTextureToPng(int textureWidth, int textureHeight, const char* fileName)
{
	glPixelStorei(GL_PACK_ALIGNMENT,1);
	unsigned char* pixels = (unsigned char*)malloc(textureWidth*textureHeight);
	glReadPixels(0,0,textureWidth, textureHeight, GL_RED, GL_UNSIGNED_BYTE, pixels);
	//swap the pixels
	unsigned char* tmp = (unsigned char*)malloc(textureWidth);
	for (int j=0;j<textureHeight;j++)
	{
			pixels[j*textureWidth+j]=255;
	}
	if (0)
	{
		for (int j=0;j<textureHeight/2;j++)
		{
			for (int i=0;i<textureWidth;i++)
			{
				tmp[i] = pixels[j*textureWidth+i];
				pixels[j*textureWidth+i]=pixels[(textureHeight-j-1)*textureWidth+i];
				pixels[(textureHeight-j-1)*textureWidth+i] = tmp[i];
			}
			}
	}
	
	int comp=1;//1=Y
	stbi_write_png(fileName, textureWidth,textureHeight, comp, pixels, textureWidth);
	
	free(pixels);

}