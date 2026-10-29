// Grasp assist for Gazebo Classic.
//
// Friction alone cannot hold objects in TIAGo's simulated gripper: the
// fingers are position controlled (gazebo_ros2_control sets their position
// directly) and PAL's finger contacts (minDepth 0.1) let them sink into
// objects, so objects slip out or the fingers close through them. This world
// plugin holds an object rigidly to the gripper while all fingers touch it,
// stops the fingers from closing further into it, and releases it when the
// fingers open again (same idea as gazebo_grasp_fix).
//
// <plugin name="grasp_assist" filename="libgrasp_assist.so">
//   <robot>tiago</robot>
//   <finger_joint>gripper_left_finger_joint</finger_joint>
//   <finger_joint>gripper_right_finger_joint</finger_joint>
//   <!-- Optional. Without <object>s, any non-static model up to max_mass
//        (kg, default 2) can be grasped, including ones spawned later. -->
//   <object>coke_kitchen</object>
//   <max_mass>2.0</max_mass>
//   <release_distance>0.005</release_distance>
// </plugin>

#include <map>
#include <set>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace simulation
{

class GraspAssist : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    robot_name_ = sdf->Get<std::string>("robot");
    for (auto e = sdf->GetElement("finger_joint"); e; e = e->GetNextElement("finger_joint")) {
      finger_joint_names_.push_back(e->Get<std::string>());
    }
    if (sdf->HasElement("object")) {
      for (auto e = sdf->GetElement("object"); e; e = e->GetNextElement("object")) {
        object_names_.insert(e->Get<std::string>());
      }
    }
    if (sdf->HasElement("max_mass")) {
      max_mass_ = sdf->Get<double>("max_mass");
    }
    if (sdf->HasElement("release_distance")) {
      release_distance_ = sdf->Get<double>("release_distance");
    }

    // Contacts are only computed when something subscribes to them
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);

    update_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&GraspAssist::OnUpdate, this));
  }

private:
  // The robot is spawned after the world loads, so look it up lazily
  bool FindRobot()
  {
    robot_ = world_->ModelByName(robot_name_);
    if (!robot_) {
      return false;
    }
    finger_joints_.clear();
    finger_links_.clear();
    for (const auto & name : finger_joint_names_) {
      auto joint = robot_->GetJoint(name);
      if (!joint) {
        gzerr << "[grasp_assist] Joint " << name << " not found\n";
        robot_.reset();
        return false;
      }
      finger_joints_.push_back(joint);
      finger_links_.insert(joint->GetChild()->GetName());
      palm_ = joint->GetParent();
    }
    gzmsg << "[grasp_assist] Watching " << finger_links_.size()
          << " fingers on " << palm_->GetName() << "\n";
    return true;
  }

  bool Graspable(const gazebo::physics::ModelPtr & model) const
  {
    if (!object_names_.empty()) {
      return object_names_.count(model->GetName()) > 0;
    }
    if (model == robot_ || model->IsStatic()) {
      return false;
    }
    double mass = 0.0;
    for (const auto & link : model->GetLinks()) {
      mass += link->GetInertial()->Mass();
    }
    return mass <= max_mass_;
  }

  double FingerOpening() const
  {
    double sum = 0.0;
    for (const auto & joint : finger_joints_) {
      sum += joint->Position(0);
    }
    return sum / finger_joints_.size();
  }

  void OnUpdate()
  {
    if (!robot_ && !FindRobot()) {
      return;
    }

    if (joint_) {
      if (FingerOpening() > attach_opening_ + release_distance_) {
        Detach();
      }
      return;
    }

    // Which fingers touch which graspable links in this step
    std::map<gazebo::physics::LinkPtr, std::set<std::string>> touching;
    auto contacts = world_->Physics()->GetContactManager();
    for (unsigned int i = 0; i < contacts->GetContactCount(); ++i) {
      auto contact = contacts->GetContact(i);
      auto link1 = contact->collision1->GetLink();
      auto link2 = contact->collision2->GetLink();
      for (auto [finger, object] : {std::make_pair(link1, link2), std::make_pair(link2, link1)}) {
        if (finger->GetModel() == robot_ && finger_links_.count(finger->GetName()) &&
          Graspable(object->GetModel()))
        {
          touching[object].insert(finger->GetName());
        }
      }
    }

    // Attach a link once all fingers have touched it for kStepsToAttach steps
    std::map<std::string, int> grip_steps;
    for (const auto & [object, fingers] : touching) {
      if (fingers.size() == finger_links_.size()) {
        const auto name = object->GetScopedName();
        grip_steps[name] = grip_steps_[name] + 1;
        if (grip_steps[name] >= kStepsToAttach) {
          Attach(object);
          return;
        }
      }
    }
    grip_steps_ = grip_steps;
  }

  void Attach(const gazebo::physics::LinkPtr & object)
  {
    // A zero-range revolute joint holds the current relative pose; this is
    // more stable in ODE than a fixed joint (as in gazebo_ros_link_attacher).
    joint_ = world_->Physics()->CreateJoint("revolute", robot_);
    joint_->Attach(palm_, object);
    joint_->Load(palm_, object, ignition::math::Pose3d());
    joint_->SetModel(robot_);
    joint_->SetUpperLimit(0, 0);
    joint_->SetLowerLimit(0, 0);
    joint_->Init();

    // Stop the fingers where they touch the object. PAL's finger contacts
    // (minDepth 0.1) let them sink into objects, so without this they keep
    // closing through the held object. Opening is still free.
    finger_lower_limits_.clear();
    for (const auto & joint : finger_joints_) {
      finger_lower_limits_.push_back(joint->LowerLimit(0));
      joint->SetLowerLimit(0, joint->Position(0));
    }

    attached_ = object->GetModel()->GetName();
    attach_opening_ = FingerOpening();
    grip_steps_.clear();
    gzmsg << "[grasp_assist] Attached " << attached_ << "\n";
  }

  void Detach()
  {
    joint_->Detach();
    joint_.reset();
    for (size_t i = 0; i < finger_joints_.size(); ++i) {
      finger_joints_[i]->SetLowerLimit(0, finger_lower_limits_[i]);
    }
    gzmsg << "[grasp_assist] Released " << attached_ << "\n";
    attached_.clear();
  }

  static constexpr int kStepsToAttach = 20;  // 20 ms at a 1 kHz physics rate

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_;
  std::string robot_name_;
  std::vector<std::string> finger_joint_names_;
  std::set<std::string> object_names_;
  double max_mass_ = 2.0;
  double release_distance_ = 0.005;

  gazebo::physics::ModelPtr robot_;
  gazebo::physics::LinkPtr palm_;
  std::vector<gazebo::physics::JointPtr> finger_joints_;
  std::set<std::string> finger_links_;
  std::vector<double> finger_lower_limits_;

  std::map<std::string, int> grip_steps_;
  std::string attached_;
  double attach_opening_ = 0.0;
  gazebo::physics::JointPtr joint_;
};

GZ_REGISTER_WORLD_PLUGIN(GraspAssist)

}  // namespace simulation
