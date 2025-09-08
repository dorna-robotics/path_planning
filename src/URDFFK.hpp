#pragma once
//#include <urdfdom_headers/urdf_model/model.h>
//#include <urdfdom/urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <sstream>

struct URDFJointStage {
    std::string parent_link;
    std::string child_link;
    urdf::JointSharedPtr joint; // can be null for fixed
};

class URDFFK {
public:
    // link_names must be the chain in order: base, ..., tip
    URDFFK(const std::string& urdf_path, const std::vector<std::string>& link_names)
        : linkNames_(link_names) {

        if (linkNames_.size() < 2)
            throw std::runtime_error("Need at least base and tip link names");

        // Load URDF
        std::string xml = readFile(urdf_path);
        model_ = urdf::parseURDF(xml);
        if (!model_) throw std::runtime_error("Failed to parse URDF: " + urdf_path);

        // Build stages (parent→child via joint)
        stages_.clear();
        stages_.reserve(linkNames_.size() - 1);
        for (size_t i = 0; i + 1 < linkNames_.size(); ++i) {
            const std::string& parent = linkNames_[i];
            const std::string& child  = linkNames_[i+1];
            // Find a joint connecting parent→child
            urdf::JointSharedPtr found;
            for (const auto& kv : model_->joints_) {
                const auto& j = kv.second;
                if (!j || !j->parent_link_name.size() || !j->child_link_name.size()) continue;
                if (j->parent_link_name == parent && j->child_link_name == child) {
                    found = j; break;
                }
            }
            if (!found) {
                // Allow fixed via link->parent_joint when child’s parent matches parent
                auto link = model_->getLink(child);
                if (!link || !link->parent_joint) {
                    throw std::runtime_error("No joint connecting " + parent + " -> " + child);
                }
                found = link->parent_joint;
                if (!(found->parent_link_name == parent && found->child_link_name == child)) {
                    throw std::runtime_error("URDF joint topology mismatch for " + parent + " -> " + child);
                }
            }
            stages_.push_back({parent, child, found});
        }

        // Count DOF
        dof_ = 0;
        for (auto& s : stages_) {
            if (!s.joint) continue;
            if (s.joint->type == urdf::Joint::REVOLUTE || s.joint->type == urdf::Joint::CONTINUOUS ||
                s.joint->type == urdf::Joint::PRISMATIC) {
                ++dof_;
            }
        }
    }

    size_t dof() const { return dof_; }
    const std::vector<std::string>& linkNames() const { return linkNames_; }

    // Compute world transforms for each link name, same order as linkNames()
    // q size must equal dof() (revolute: radians; prismatic: meters)
    void compute(const Eigen::VectorXd& q, std::vector<Eigen::Isometry3d>& linkWorld , const Eigen::Isometry3d& base_trans) const {
        if ((size_t)q.size() != dof_) throw std::runtime_error("q size != dof");

        linkWorld.resize(linkNames_.size());
        Eigen::Isometry3d T = base_trans;
        linkWorld[0] = T; // base

        size_t qi = 0;
        for (size_t i = 0; i < stages_.size(); ++i) {
            const auto& st = stages_[i];
            const auto& j  = st.joint;

            // Joint origin (pose of joint frame in parent link frame)
            Eigen::Isometry3d To = Eigen::Isometry3d::Identity();
            if (j) {
                const auto& o = j->parent_to_joint_origin_transform;
                To.translate(Eigen::Vector3d(o.position.x, o.position.y, o.position.z));
                // URDF uses RPY for origin rotation
                const urdf::Rotation& rot = o.rotation;
                Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);  // note order: w,x,y,z
                To.linear() = q.toRotationMatrix();
            }

            // Motion due to joint value
            Eigen::Isometry3d Tj = Eigen::Isometry3d::Identity();
            if (j) {
                if (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS) {
                    Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
                    if (axis.norm() == 0) axis = Eigen::Vector3d::UnitZ();
                    axis.normalize();
                    Tj.linear() = Eigen::AngleAxisd(q[qi++], axis).toRotationMatrix();
                } else if (j->type == urdf::Joint::PRISMATIC) {
                    Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
                    if (axis.norm() == 0) axis = Eigen::Vector3d::UnitZ();
                    axis.normalize();
                    Tj.translate(axis * q[qi++]);
                } else {
                    // FIXED or others: no dof increment
                }
            }

            // World transform of child link
            T = T * To * Tj;
            linkWorld[i+1] = T;
        }
    }

    // Optional: extract joint limits (min,max) for revolute/prismatic
    std::vector<std::pair<double,double>> jointLimits() const {
        std::vector<std::pair<double,double>> out;
        out.reserve(dof_);
        for (auto& st : stages_) {
            const auto& j = st.joint;
            if (!j) continue;
            if (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::PRISMATIC) {
                if (j->limits) out.emplace_back(j->limits->lower, j->limits->upper);
                else out.emplace_back(-3.14159265, 3.14159265); // fallback
            } else if (j->type == urdf::Joint::CONTINUOUS) {
                out.emplace_back(-3.14159265, 3.14159265);
            }
        }
        return out;
    }

    const urdf::ModelInterfaceSharedPtr& model() const { return model_; }

private:
    static std::string readFile(const std::string& path) {
        std::ifstream ifs(path, std::ios::in | std::ios::binary);
        if (!ifs) throw std::runtime_error("Cannot open: " + path);
        std::ostringstream ss; ss << ifs.rdbuf(); return ss.str();
    }

    urdf::ModelInterfaceSharedPtr model_;
    std::vector<std::string> linkNames_;
    std::vector<URDFJointStage> stages_;
    size_t dof_{0};
};
