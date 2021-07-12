#include <unordered_map>
#include <string>
#include <vector>

float calculateInitialPos(int index, float offset, float size, int resolution);

namespace multiple_abb_irb120{
    
    class Grid{
    private:
        ros::NodeHandle* nh;
        float x, y, w, h;
        int hr, vr;
        std::vector<std::vector<GazeboSphere> > grid;
    public:
        Grid(ros::NodeHandle* nh, float x_origin, float y_origin, float width, float height, int hres, int vres);
        const GazeboSphere& getSphere(int i, int j);
        void update();
    };
}
