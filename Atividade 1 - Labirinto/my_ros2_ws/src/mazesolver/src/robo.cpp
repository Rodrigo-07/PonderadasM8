#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_set>
#include <memory>
#include <random>

using namespace std;
using namespace chrono_literals;

class MoveMazeSolverClient : public rclcpp::Node {
public:
    MoveMazeSolverClient() 
        : Node("move_maze_solver_client"), 
          gerador_(random_device{}())
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    // Vector é um array dinâmico que pode ser redimensionado porem aceita apenas um tipo de dado, não igual list em python
    vector<string> send_request(const string &direction) {
        RCLCPP_INFO(this->get_logger(), "Enviando requisição para direção: %s", direction.c_str());

        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Serviço '/move_command' não disponível");
            return {};
        }

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao receber resposta.");
            return {};
        }

        auto response = future.get();

        RCLCPP_INFO(this->get_logger(), "Resposta recebida: Sucesso=%d, Posição do Robô=[%d, %d], Posição do Alvo=[%d, %d]",
                    response->success, response->robot_pos[0], response->robot_pos[1], response->target_pos[0], response->target_pos[1]);

        save_command(direction, response->success, response->robot_pos, response->target_pos, response->left, response->down, response->up, response->right);

        vector<string> final_response;
        final_response.push_back(response->success ? "true" : "false");
        final_response.push_back(response->left);
        final_response.push_back(response->down);
        final_response.push_back(response->up);
        final_response.push_back(response->right);

        RCLCPP_INFO(this->get_logger(), "Resposta final: %s, %s, %s, %s, %s", final_response[0].c_str(), final_response[1].c_str(), final_response[2].c_str(), final_response[3].c_str(), final_response[4].c_str());

        return final_response;
    }

    unordered_set<string> get_empty_positions(const string &left, const string &down, const string &up, const string &right) {
        unordered_set<string> emptyPositions;

        if(left == "f"){
            emptyPositions.insert("left");
        }

        if(down == "f"){
            emptyPositions.insert("down");
        }

        if(up == "f"){
            emptyPositions.insert("up");
        }

        if(right == "f"){
            emptyPositions.insert("right");
        }

        return emptyPositions;
    }

    string realizar_sorteio(const unordered_set<string>& conjunto) {
        // Converte o unordered_set para um vector para facilitar o acesso por índice
        vector<string> elementos(conjunto.begin(), conjunto.end());

        // Gera um número aleatório entre 0 e o número de elementos do conjunto - 1
        uniform_int_distribution<size_t> distribuicao(0, elementos.size() - 1);

        // Seleciona o elemento sorteado usando o gerador inicializado no construtor
        size_t indice_sorteado = distribuicao(gerador_);

        RCLCPP_INFO(this->get_logger(), "Posição sorteada : %s", elementos[indice_sorteado].c_str());

        return elementos[indice_sorteado];
    }

    string checkTargetPostion(const string &left, const string &down, const string &up, const string &right) {
        if(left == "t"){
            return "left";
        }

        if(down == "t"){
            return "down";
        }

        if(up == "t"){
            return "up";
        }

        if(right == "t"){
            return "right";
        }

        return "";
    }

    void randomNavigation() {
        bool targetReached = false;

        int moves = 0;

        vector<string> moveResult;

        moveResult = send_request("right");

        moves++;

        while(targetReached == false){

            // Pegar as posições vazias e sortear uma
            unordered_set<string> emptyPositions;
            string TargetFoud = checkTargetPostion(moveResult[1], moveResult[2], moveResult[3], moveResult[4]);

            if(TargetFoud == "left" || TargetFoud == "down" || TargetFoud == "up" || TargetFoud == "right"){
                RCLCPP_INFO(this->get_logger(), "Alvo encontrado na posição: %s", TargetFoud.c_str());
                targetReached = true;
                send_request(TargetFoud);
                moves++;
                break;
            }
            // O resultado vem com 4 opções (left, down, up, right) e cada um pode ter um dos valores t (target), r(robo), f(free) e b (blocked)

            emptyPositions = get_empty_positions(moveResult[1], moveResult[2], moveResult[3], moveResult[4]);

            string emptyPositionsString = "";

            for(auto it = emptyPositions.begin(); it != emptyPositions.end(); ++it){
                emptyPositionsString += *it + " ";
            }

            RCLCPP_INFO(this->get_logger(), "Posições vazias: %s", emptyPositionsString.c_str());
            
            // sortear uma posição
            string randomPosition = realizar_sorteio(emptyPositions);

            usleep(100);

            moveResult = send_request(randomPosition);
            moves++;

            RCLCPP_INFO(this->get_logger(), "Posição sorteada: %s", randomPosition.c_str());
            
            usleep(100);
        }

        RCLCPP_INFO(this->get_logger(), "Alvo encontrado em %d movimentos", moves);

        // rclcpp::shutdown();
        return;
    }

private:
    void save_command(const string &direction, bool success, const array<int8_t, 2> &robot_pos, const array<int8_t, 2> &target_pos, const string &left, const string &down, const string &up, const string &right) {
        history_.emplace_back(direction, success, robot_pos, target_pos, left, down, up, right);
    }

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    vector<tuple<string, bool, array<int8_t, 2>, array<int8_t, 2>, string, string, string, string>> history_;
    default_random_engine gerador_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = make_shared<MoveMazeSolverClient>();

    node->send_request("none");
    usleep(100000);

    node->send_request("none");
    usleep(100000);

    node->send_request("left");
    usleep(100000);

    node->send_request("right");
    usleep(100000);

    // node->randomNavigation();


    rclcpp::shutdown();
    return 0;
}
