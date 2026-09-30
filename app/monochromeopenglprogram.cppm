module;
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <map>
#include <string>

export module retopo.app.monochrome_opengl_program;

export class MonochromeOpenGLProgram : public QOpenGLShaderProgram {
public:
    void load(bool isCoreProfile = false);
    int getUniformLocationByName(const std::string& name);
    bool isCoreProfile() const;

private:
    bool m_isLoaded = false;
    bool m_isCoreProfile = false;
    std::map<std::string, int> m_uniformLocationMap;
};
