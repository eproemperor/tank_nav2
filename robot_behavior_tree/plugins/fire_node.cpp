#include "fire_node.h"

namespace fire_node
{
    Fire::Fire(const std::string &action_name,
               const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf) {}

    double Fire::calangle(double &x1, double &y1, double &x2, double &y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double angle_rad = atan2(dy, dx);
        double angle_deg = angle_rad * 180.0 / M_PI;

        if (angle_deg < 0)
        {
            angle_deg += 360.0;
        }
        if (angle_deg >= 0 && angle_deg < 5)
        {
            angle_deg = 0; // 右
        }
        else if (angle_deg >= 85 && angle_deg < 95)
        {
            angle_deg = 90; // 上
        }
        else if (angle_deg >= 175 && angle_deg < 185)
        {
            angle_deg = 180; // 左
        }
        else if (angle_deg >= 265 && angle_deg < 275)
        {
            angle_deg = 270; // 下
        }
        else if (angle_deg >= 355 && angle_deg < 360)
        {
            angle_deg = 0; // 右
        }
        return angle_deg;
    }

    void Fire::fire(double theta)
    {
        RobotMsgProcess_.SendFireCommand(theta);
    };

    void Fire::updateposition()
    {
        getInput<TypeMode>("sentry_info", sentry);
        getInput<TypeMode>("enemy_base_info", enemy_base);
        getInput<TypeMode>("enemy_info", enemy_base);
        getInput<int>("enemy_num", enemy_num);
    };

    BT::NodeStatus Fire::tick()
    {
        if (RobotMsgProcess_.open())
        {
            updateposition();
            if (!enemy_num == 0)
            {
                theta = calangle(sentry.p_x, sentry.p_y, enemy.p_x, enemy.p_y);
            }
            else
            {
                theta = calangle(sentry.p_x, sentry.p_y, enemy_base.p_x, enemy_base.p_y);
            }
            fire(theta);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    };

}