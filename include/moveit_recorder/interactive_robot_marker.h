#include <interactive_markers/interactive_marker_server.h>
#include <Eigen/Geometry>


/* Interactive marker.
 *
 * Constructing creates an interactive marker with either 3 or 6 DOF controls.
 *
 * name - name of marker and text of marker description
 * pose - starting position of marker
 * callback - called when marker is moved
 * dof - POS = 3DOF positional, ORIENT = 3DOF orientation, BOTH = 6DOF
 */
class IMarker
{
  public:
    /* degrees of freedom (3d position, 3d orientation, or both) */
    enum Dof { BOTH, POS, ORIENT, PLANAR };

    /** create an interactive marker at the origin */
    IMarker(interactive_markers::InteractiveMarkerServer& server,
        const std::string& name,
        const std::string& frame_id = "/base_footprint",
        boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback = printFeedback,
        Dof dof = BOTH) :
      imarker_(),
      dof_(dof)
  {
    initialize(server, name, Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0), frame_id, callback, dof);
  }

    /** create an interactive marker with an initial pose */
    IMarker(interactive_markers::InteractiveMarkerServer& server,
        const std::string& name,
        const Eigen::Affine3d& pose,
        const std::string& frame_id = "/base_footprint",
        boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback = printFeedback,
        Dof dof = BOTH) :
      imarker_(),
      dof_(dof)
  {
    Eigen::Quaterniond q(pose.linear());
    Eigen::Vector3d p = pose.translation();

    initialize(server, name, p, q, frame_id, callback, dof);
  }

    /** create an interactive marker with an initial pose */
    IMarker(interactive_markers::InteractiveMarkerServer& server,
        const std::string& name,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const std::string& frame_id = "/base_footprint",
        boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback = printFeedback,
        Dof dof = BOTH) :
      imarker_(),
      dof_(dof)
  {
    initialize(server, name, position, orientation, frame_id, callback, dof);
  }

    /** create an interactive marker with an initial position */
    IMarker(interactive_markers::InteractiveMarkerServer& server,
        const std::string& name,
        const Eigen::Vector3d& position,
        const std::string& frame_id = "/base_footprint",
        boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback = printFeedback,
        Dof dof = BOTH) :
      imarker_(),
      dof_(dof)
  {
    initialize(server, name, position, Eigen::Quaterniond(1,0,0,0), frame_id, callback, dof);
  }

    /** move marker to new pose */
    void move(const Eigen::Affine3d& pose);

    /** default callback which just prints new position and orientation */
    static void printFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  private:

    /* called by constructors */
    void initialize(
        interactive_markers::InteractiveMarkerServer& server,
        const std::string& name,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const std::string& frame_id,
        boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback,
        Dof dof);

    enum Axis { X, Y, Z };
    visualization_msgs::Marker makeAxisCyl( Axis axis );
    visualization_msgs::Marker makeBall();
    void makeBallControl();
    void makeAxisControl();
    void makePlanarControl();
    void setControls(Dof dof);

    visualization_msgs::InteractiveMarker imarker_;
    interactive_markers::InteractiveMarkerServer* server_;
    Dof dof_;
};
