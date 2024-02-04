#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

#include "customs/srv/matrix_exponential_discretization_by_accuracy.hpp"

class StateSpaceNode : public rclcpp::Node
{
public:
    StateSpaceNode() : Node("state_space_node")
    {
        this->declare_parameter("name", "StateSpaceNode");
        this->declare_parameter("server_wait", 1.0);
        this->declare_parameter("accuracy", 0.001);
        this->declare_parameter("frequency", 100.0);

        _name = this->get_parameter("name").as_string();
        _server_wait = this->get_parameter("server_wait").as_double();
        _accuracy = this->get_parameter("accuracy").as_double();
        _frequency = this->get_parameter("frequency").as_double();
        _dt = 1.0 / _frequency;

        _threads.push_back(std::thread(std::bind(&StateSpaceNode::call_MatrixExponentialDiscretizationByAccuracy, this, A, B, _dt, _accuracy)));

        _time_publisher = this->create_publisher<example_interfaces::msg::Float64>(
            "time",
            10);

        _rate_publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "rate",
            10);

        _output_publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "output",
            10);

        _state_eul_publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "state_eul",
            10);

        _state_med_publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "state_med",
            10);

        _input_publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "input",
            10);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / _frequency)),
            std::bind(&StateSpaceNode::_callback_timer, this));

        RCLCPP_INFO(this->get_logger(), "%s has been started.", _name.c_str());
    }

private:
    std::string _name = "";

    double _frequency = 0.0;
    double _time = 0.0;
    double _dt = 0.0;
    double _accuracy = 0.0;
    double _server_wait = 0.0;

    std::vector<std::thread> _threads;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr _time_publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _rate_publisher;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _output_publisher;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _state_eul_publisher;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _state_med_publisher;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _input_publisher;

    double m = 1.0;
    double b = 1.0;
    double k = 10.0;
    double f = 1.0;

    // velocity / position
    // X_D = A * X + B * U
    // Y = C * X + D * U
    double xd_1 = 0.0;
    double xd_2 = 0.0;

    double y_1 = 0.0;
    double y_2 = 0.0;

    std::vector<double> X_D = {xd_1, xd_2};
    std::vector<double> Y = {y_1, y_2};
    std::vector<double> X_EUL = {0.0, 0.0};
    std::vector<double> X_MED = {0.0, 0.0};
    std::vector<double> U = {f};

    double a_0_0 = -b / m;
    double a_0_1 = -k / m;
    double a_1_0 = 1.0;
    double a_1_1 = 0.0;

    std::vector<std::vector<double>> A = {{a_0_0, a_0_1},
                                          {a_1_0, a_1_1}};

    double b_0_0 = 1.0 / m;
    double b_1_0 = 0.0;

    std::vector<std::vector<double>> B = {{b_0_0},
                                          {b_1_0}};

    double c_0_0 = 1.0;
    double c_0_1 = 0.0;
    double c_1_0 = 0.0;
    double c_1_1 = 1.0;

    std::vector<std::vector<double>> C = {{c_0_0, c_0_1},
                                          {c_1_0, c_1_1}};

    double d_0_0 = 0.0;
    double d_1_0 = 0.0;

    std::vector<std::vector<double>> D = {{d_0_0},
                                          {d_1_0}};

    double f_0_0 = 0.0;
    double f_0_1 = 0.0;
    double f_1_0 = 0.0;
    double f_1_1 = 0.0;

    std::vector<std::vector<double>> F = {{f_0_0, f_0_1},
                                          {f_1_0, f_1_1}};

    double g_0_0 = 0.0;
    double g_1_0 = 0.0;

    std::vector<std::vector<double>> G = {{g_0_0},
                                          {g_1_0}};

    void _callback_timer()
    {
        _time += _dt;

        _time_publish();

        _calculate_state_discrete_euler_approximation();
        _calculate_state_discrete_exponential_discretization();

        _state_publish();
    }

    // Euler Approximation method
    void _calculate_state_discrete_euler_approximation()
    {
        X_D[0] = A[0][0] * X_EUL[0] + A[0][1] * X_EUL[1] + B[0][0] * U[0];
        X_D[1] = A[1][0] * X_EUL[0] + A[1][1] * X_EUL[1] + B[1][0] * U[0];

        Y[0] = C[0][0] * X_EUL[0] + C[0][1] * X_EUL[1] + D[0][0] * U[0];
        Y[1] = C[1][0] * X_EUL[0] + C[1][1] * X_EUL[1] + D[1][0] * U[0];

        X_EUL[0] += X_D[0] * _dt;
        X_EUL[1] += X_D[1] * _dt;
    }

    // Matrix Exponential Discretization method
    void _calculate_state_discrete_exponential_discretization()
    {
        double X_0 = F[0][0] * X_MED[0] + F[0][1] * X_MED[1] + G[0][0] * U[0];
        double X_1 = F[1][0] * X_MED[0] + F[1][1] * X_MED[1] + G[1][0] * U[0];

        Y[0] = C[0][0] * X_MED[0] + C[0][1] * X_MED[1] + D[0][0] * U[0];
        Y[1] = C[1][0] * X_MED[0] + C[1][1] * X_MED[1] + D[1][0] * U[0];

        X_MED[0] = X_0;
        X_MED[1] = X_1;
    }

    void _state_publish()
    {
        auto rate = example_interfaces::msg::Float64MultiArray();
        rate.data = X_D;

        auto output = example_interfaces::msg::Float64MultiArray();
        output.data = Y;

        auto state_eul = example_interfaces::msg::Float64MultiArray();
        state_eul.data = X_EUL;

        auto state_med = example_interfaces::msg::Float64MultiArray();
        state_med.data = X_MED;

        auto input = example_interfaces::msg::Float64MultiArray();
        input.data = U;

        _rate_publisher->publish(rate);
        _output_publisher->publish(output);
        _state_eul_publisher->publish(state_eul);
        _state_med_publisher->publish(state_med);
        _input_publisher->publish(input);
    }

    void _time_publish()
    {
        auto msg = example_interfaces::msg::Float64();
        msg.data = _time;
        _time_publisher->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Time:[%lf]", _time);
    }

    std::vector<std::vector<double>> _construct_matrix(const std::vector<double> a, const size_t n, const size_t m)
    {
        if (a.size() != (n * m))
        {
            return {{0.0}};
        }

        size_t index = 0;
        std::vector<std::vector<double>> A;

        for (size_t i = 1; i <= n; i++)
        {
            std::vector<double> row;

            for (size_t j = 1; j <= m; j++)
            {
                row.push_back(a[index++]);
            }

            A.push_back(row);
        }

        return A;
    }

    std::vector<double> _deconstruct_matrix(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();

        std::vector<double> a;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                a.push_back(A[i][j]);
            }
        }

        return a;
    }

    void call_MatrixExponentialDiscretizationByAccuracy(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B, const double dt, const double accuracy)
    {
        int n1 = A.size();
        int m1 = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        int n2 = B.size();
        int m2 = B[0].size();
        std::vector<double> b = _deconstruct_matrix(B);

        auto client = this->create_client<customs::srv::MatrixExponentialDiscretizationByAccuracy>("matrix_exponential_discretization_by_accuracy");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixExponentialDiscretizationByAccuracy::Request>();

        request->system_matrix = a;
        request->n1 = n1;
        request->m1 = m1;

        request->input_matrix = b;
        request->n2 = n2;
        request->m2 = m2;

        request->dt = dt;
        request->accuracy = accuracy;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            F = _construct_matrix(response->result_system, n1, m1);
            G = _construct_matrix(response->result_input, n2, m2);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateSpaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
