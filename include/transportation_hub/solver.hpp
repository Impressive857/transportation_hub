#ifndef _SOLVER_H_
#define _SOLVER_H_

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "judger_interfaces/msg/overall_info.hpp"
#include "judger_interfaces/msg/my_answer.hpp"
#include "judger_interfaces/srv/my_service.hpp"

class Solver
    :public rclcpp::Node
{
    using Question = judger_interfaces::msg::OverallInfo; //问题类型
    using Answer = judger_interfaces::msg::MyAnswer;      //答案类型

    using Server = judger_interfaces::srv::MyService;     //服务器类型
public:
    explicit Solver(const std::string& nodeName);
    Solver(const Solver&) = delete;
    Solver& operator=(const Solver&) = delete;
private:
    /// @brief 获取问题回调函数
    /// @param question 通过question话题获取的问题指针
    void cbfn_solve(const Solver::Question::SharedPtr question);
    /// @brief floyd寻路算法
    void floyd();
private:
    /// @brief 获取两个城市距离
    /// @param src 出发城市
    /// @param dst 终点城市
    /// @return 两个城市之间的距离
    /// @retval int32_t len 距离
    /// @retval int32_t (-1) 没有道路
    int32_t get_length(int32_t src, int32_t dst);
    /// @brief 获取两个城市中间点
    /// @param src 出发城市
    /// @param dst 终点城市
    /// @return 两个城市之间的中间点
    /// @retval int32_t city 中间点城市
    /// @retval int32_t (-1) 没有道路
    int32_t get_path(int32_t src, int32_t dst);
private:
    rclcpp::Subscription<Question>::SharedPtr m_question;//
    rclcpp::Client<Server>::SharedPtr m_client;//

    std::unordered_map<int32_t, std::unordered_map<int32_t, int32_t>> m_city_map; //存储城市信息
    std::vector<std::vector<int32_t>> m_len_mat;                                  //长度矩阵
    std::vector<std::vector<int32_t>> m_path_mat;                                 //路径矩阵
    Solver::Answer m_answer;                                                      //答案
    int32_t m_src_city;                                                           //起始城市
    int32_t m_dst_city;                                                           //终点城市
    int32_t m_city_num;                                                           //城市数量
};
#endif // ^^ !_SOLVER_H_