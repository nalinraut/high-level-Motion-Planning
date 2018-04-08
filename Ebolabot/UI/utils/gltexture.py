from OpenGL.GL import *

#imaging stuff
try:
    from PIL import Image
except ImportError, err:
    import Image

class GLTexture:
    def __init__(self,fn=None):
        self.glid = None
        if fn:
            self.loadImage(fn)
    def destroy(self):
        glDeleteTextures([self.glid])

    def setBytes(self,w,h,buffer,glformat=GL_RGBA):
        self.w,self.h = w,h
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, glformat, w, h, 0,
            glformat, GL_UNSIGNED_BYTE, buffer
        )
    def loadImage(self,fn):
        im = Image.open(fn)
        try:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBA",0,-1)
        except SystemError:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBX",0,-1)
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, self.w, self.h, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, image
        )
        return True
    def enable(self,smooth=True,glmode=GL_MODULATE):
        glEnable(GL_TEXTURE_2D)
        if smooth:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        else:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, glmode)
        glBindTexture(GL_TEXTURE_2D,self.glid)
    def disable(self):
        glDisable(GL_TEXTURE_2D)
    def blit(self,x,y,w=None,h=None):
        if w==None: w = self.w
        if h==None: h = self.h
        self.enable()
        glDisable(GL_LIGHTING)
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1)
        glVertex2f(x,y)
        glTexCoord2f(0,0)
        glVertex2f(x,y+h)
        glTexCoord2f(1,0)
        glVertex2f(x+w,y+h)
        glTexCoord2f(1,1)
        glVertex2f(x+w,y)
        glEnd()
        self.disable()
