#include "OgreApplicationContext.h"
#include "OgreApplicationContextBase.h"
#include "OgreRenderWindow.h"
#include "OgreRoot.h"

namespace visualizer {

class MyTestApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    MyTestApp();
    void setup();
    bool keyPressed(const OgreBites::KeyboardEvent& evt);
};

}  // namespace visualizer