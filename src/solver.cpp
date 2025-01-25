#include "solver.hpp"

Solver::Solver(const std::string& nodeName)
    :rclcpp::Node(nodeName)
{
    m_question = create_subscription<Solver::Question>("question", 1, std::bind(&Solver::cbfn_solve, this, std::placeholders::_1));

    m_client = create_client<Solver::Server>("judger_server");
}

void Solver::floyd() {
    for (int32_t i = 0;i < m_city_num;i++) {
        for (int32_t j = 0;j < m_city_num;j++) {
            for (int32_t k = 0; k < m_city_num;k++) {
                // 去除floyd算法每次迭代不变的地方
                if (i != j && i != k && j != k) {
                    // 每次迭代， i为中间点， 获取两段路径长度
                    int32_t len_1 = get_length(j, i);
                    int32_t len_2 = get_length(i, k);
                    // 如果路径不存在，两条路径加起来也不存在
                    int32_t len = (len_1 < 0 || len_2 < 0) ? (-1) : (len_1 + len_2);
                    //如果路径存在
                    if (len >= 0) {
                        //更新最短路径
                        if (m_len_mat[j][k] > len || -1 == m_len_mat[j][k]) {
                            m_len_mat[j][k] = len;
                            //新的中间点是后半段路径的中间点
                            m_path_mat[j][k] = m_path_mat[i][k];
                        }
                    }
                }
            }
        }
    }
}

void Solver::cbfn_solve(const Solver::Question::SharedPtr question) {
    RCLCPP_INFO(get_logger(), "number of cities:%d", question->number_of_cities);
    RCLCPP_INFO(get_logger(), "number of roads:%d", question->number_of_roads);
    RCLCPP_INFO(get_logger(), "src city:%d", question->src_city);
    RCLCPP_INFO(get_logger(), "dst city:%d", question->des_city);

    // 初始化城市数据
    for (auto& info : question->infos) {
        RCLCPP_INFO(get_logger(), "src:%d, dst:%d, length:%d", info.source, info.destination, info.length);
        m_city_map[info.source][info.destination] = info.length;
        m_city_map[info.destination][info.source] = info.length;
    }
    m_src_city = question->src_city;
    m_dst_city = question->des_city;
    m_city_num = question->number_of_cities;

    // 初始化两个矩阵
    m_len_mat.resize(m_city_num);
    m_path_mat.resize(m_city_num);
    for (int32_t i = 0;i < m_city_num;i++) {
        m_len_mat[i].resize(m_city_num);
        m_path_mat[i].resize(m_city_num);
    }
    for (int32_t i = 0;i < m_city_num;i++) {
        for (int32_t j = 0;j < m_city_num;j++) {
            m_len_mat[i][j] = get_length(i, j);
            m_path_mat[i][j] = get_path(i, j);
        }
    }

    floyd();// floyd算法寻找路径
    m_answer.my_answer.clear();//必须clear，要不然每一次被判题机调用就会多数据
    m_answer.my_answer.push_back(m_dst_city);//先放入终点城市
    for (int32_t mid = m_path_mat[m_src_city][m_dst_city]; mid != -1;) {//寻找中间的路经城市
        if (mid != m_src_city && mid != -1) {//如果中间城市就是起始城市就不加入
            m_answer.my_answer.push_back(mid);
        }
        mid = m_path_mat[m_src_city][mid];//更新中间点
    }
    m_answer.my_answer.push_back(m_src_city);//最后放入起始城市
    std::reverse(m_answer.my_answer.begin(), m_answer.my_answer.end()); //翻转以获得正确顺序
    std::stringstream ss;
    for (auto& city : m_answer.my_answer) {
        ss << city << "->";
    }
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    auto request = std::make_shared<Server::Request>();
    request->answer = m_answer;
    m_client->async_send_request(request);
}

int32_t Solver::get_length(int32_t src, int32_t dst) {
    if (m_city_map.count(src) == 0) {
        return -1;
    }
    if (m_city_map[src].count(dst) == 0) {
        return -1;
    }
    // 城市自己到自己的距离为0
    if (src == dst) {
        return 0;
    }
    return m_city_map[src][dst];
}

int32_t Solver::get_path(int32_t src, int32_t dst) {
    if (m_city_map.count(src) == 0) {
        return -1;
    }
    if (m_city_map[src].count(dst) == 0) {
        return -1;
    }
    // 城市自己到自己的没有路径
    if (src == dst) {
        return -1;
    }
    // 如果直接连接，中间点就是src
    return src;
}