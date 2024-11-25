#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_set>
#include <queue>
#include <set>
#include <memory>
#include <random>
#include <map>
#include <sstream>
#include <algorithm>
#include <functional>
#include <stack>

using namespace std;
using namespace chrono_literals;

class MoveMazeSolverClient : public rclcpp::Node {
public:
    MoveMazeSolverClient()
        : rclcpp::Node("move_maze_solver_client"), map_(20, vector<char>(20, 'u')), target_position_{-1, -1}
        // Incializar nó do cliente, mapa 20x20 com u de unkown, e posição do alvo como (-1, -1) já que sabemos o avo
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    // Metodo para enviar requests no serviço e já devolver a resposta processada
    vector<string> send_request(const string &direction) { // vector é como fosse uma lista em python só que com apenas um tipo de dado
        RCLCPP_INFO(this->get_logger(), "Enviando requisição para direção: %s", direction.c_str());

        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Serviço não disponivel");
            return {};
        }

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future = client_->async_send_request(request); // Enviar de forma assincrona o request

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao receber resposta.");
            return {};
        }

        auto response = future.get();

        RCLCPP_INFO(this->get_logger(), "Resposta recebida: Sucesso=%d, Posição do Robô=[%d, %d], Posição do Alvo=[%d, %d]",
                    response->success, response->robot_pos[0], response->robot_pos[1], response->target_pos[0], response->target_pos[1]);

        // Decompor toda as infos na resposta que eu preciso
        vector<string> final_response;
        final_response.push_back(response->success ? "true" : "false");
        final_response.push_back(response->left);
        final_response.push_back(response->down);
        final_response.push_back(response->up);
        final_response.push_back(response->right);
        final_response.push_back(to_string(response->robot_pos[0]));
        final_response.push_back(to_string(response->robot_pos[1]));

        RCLCPP_INFO(this->get_logger(), "Resposta final: %s, %s, %s, %s, %s, %s, %s", final_response[0].c_str(), final_response[1].c_str(), final_response[2].c_str(), final_response[3].c_str(), final_response[4].c_str(), final_response[5].c_str(), final_response[6].c_str());

        return final_response;
    }

            
    // Aqui vai ser a logica principal da bfs
    // Inspirações:
    // - https://github.com/seanperfecto/BFS-DFS-Pathfinder
    // - https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3
    // https://www.programiz.com/dsa/graph-bfs
    // https://github.com/ra101/Maze-Solver-Cpp/blob/master/bfs.h

    // logica
    // Criar fila de exploração para guardar as posições que eu vou explorar
    // Adicionar a posição inicial na fila de exploração
    // Enquanto a fila estivercheia
        // Pego o topo da fila
        // Essa posição vai ser a posição atual
        // Remover a posição atual da fila de exploração pq ele já foi explorado já qie ele está nela
        // Verifico os vizinhos da posição atual
        // Para cada vizinho eu verifico se ele já foi visitado se sim eu continuo
        // Se ele não foi visitado eu verifico se ele é livre e ser for livre marco como visitado e add na fila de exploração
        // Como eu não quero que ele pare quando ele encontrar o alvo eu vou continuar explorando msm que ele encontre o alvo
    void bfs_nav() {
        set<pair<int, int>> visited_positions; // Set para eu não precisar verificando se a posição já foi visitada
        // vector<string> moveResult = send_request("none");
        vector<string> moveResult = send_request("right");


        int start_x = stoi(moveResult[5]);
        int start_y = stoi(moveResult[6]);

        // Pair é como se fosse uma tupla em python
        pair<int, int> start_position = {start_x, start_y};

        // FIla top
        queue<pair<int, int>> exploration_queue;
        exploration_queue.push(start_position); // push vai add um elemento no final da fila

        // Dicionario para armazer o caminho para cada posição, com um pair que é a posição e um vector de string que é o caminho    
        map<pair<int, int>, vector<string>> paths;
        paths[start_position] = {};

        visited_positions.insert(start_position);
        map_[start_x][start_y] = 'f';


        while (!exploration_queue.empty()) {

            // Obter a posição atual da fila de exploração
            auto current_position = exploration_queue.front();

            // Remover a posição atual da fila de exploração pq ele já foi explorado já qie ele está nela
            exploration_queue.pop();

            // Aqui eu preciso pegar o caminho que já passei até a posição atual pq eu vou voltar para a posição inicial antes de explorar o próximo nó 
            // pq como estamos na logica de bfs eu preciso voltar para a posição que eu estava antes de epxlorar a proxima camada
            auto current_path = paths[current_position]; // então aqui em paths eu salvoo caminho para cada posição

            // MOrever o robô para a posição atual
            move_along_path(current_path);

            // OBter como está as proximas posições
            vector<string> moveResult = send_request("none");
            usleep(10000);

            vector<string> directions = {"left", "down", "up", "right"};


            vector<pair<int, int>> deltas = {{0, -1}, {1, 0}, {-1, 0}, {0, 1}};

            for (size_t i = 0; i < directions.size(); ++i) { 

                int new_x = current_position.first + deltas[i].first; // -> aqui eu pego a posicao minha e somo com o detla para saber a proxima posicao
                int new_y = current_position.second + deltas[i].second;

                if (new_x < 0 || new_y < 0 || new_x >= 20 || new_y >= 20) { // verificação para ver se vai sair do mapa
                    continue;
                }

                pair<int, int> neighbor_position = {new_x, new_y}; // posição do vizinho

                if (visited_positions.find(neighbor_position) != visited_positions.end()) { // aqui eu verifico se a posição já foi visitada se sim eu continuo
                    continue;
                }

                char status = moveResult[i + 1][0];

                // if (status == 'f' || status == 't') { // se a posição for livre ou o alvo eu vou explorar
                //     // Adicionar à fila de exploração
                //     exploration_queue.push(neighbor_position);
                //     visited_positions.insert(neighbor_position);
                //     map_[nx][ny] = status;

                //     // Atualizar o caminho para o vizinho
                //     auto neighbor_path = current_path;
                //     neighbor_path.push_back(directions[i]);
                //     paths[neighbor_position] = neighbor_path;

                //     // Se encontrar o alvo, armazenar sua posição
                //     // if (status == 't') {
                //     //     target_position_ = neighbor_position;
                //     // }

                //     // Se eu encontrar o alvo eu vou consiele como parade e vou continuar explorando
                //     if (status == 't') {
                //         map_[nx][ny] = 'b';
                //     }
                // } else if (status == 'b') {
                //     map_[nx][ny] = 'b';
                //     visited_positions.insert(neighbor_position);
                // }

                if (status == 'f') {
                    // Adicionar à fila de exploração
                    exploration_queue.push(neighbor_position);
                    visited_positions.insert(neighbor_position);
                    map_[new_x][new_y] = status;

                    // Atualizar o caminho para o vizinho
                    auto neighbor_path = current_path;
                    neighbor_path.push_back(directions[i]);
                    paths[neighbor_position] = neighbor_path;

                } else if (status == 't') {
                    // Armazenar a posição do alvo
                    target_position_ = neighbor_position;
                    map_[new_x][new_y] = 't';
                    visited_positions.insert(neighbor_position);

                    // Não posso add na fila de exploração pq eu já encontrei o alvo e ele não pode ir para esse bloco

                } else if (status == 'b') {
                    map_[new_x][new_y] = 'b';
                    visited_positions.insert(neighbor_position);
                }

                // paths 
                for (const auto& path : paths) {
                    RCLCPP_INFO(this->get_logger(), "Path para [%d, %d]: %s", path.first.first, path.first.second, path.second.empty() ? "Nenhum" : path.second.back().c_str());
                }
            }

            // Vamo retornar a posição lá do incio antes antes de explorar a proxima camada pq não consegui salvar nós de interseção e voltar para eles
            move_along_path(reverse_path(current_path));

            print_map();
        }


        move_along_path({});

        print_map();

        // usleep(10000);
        usleep(5000000);

        if (target_position_.first == -1) {
            RCLCPP_ERROR(this->get_logger(), "Posição do alvo não encontrada durante a exploração.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Vamos começar a busca pelo o melhor caminho");

        // Encontrar o melhor caminho até o alvo
        auto path_to_target = find_path_dfs(start_position, target_position_);

        if (path_to_target.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível encontrar um caminho.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Caminho encontrado");
            follow_path(path_to_target);
        }

        usleep(10000);
    }

    // funcao para mover o robÕ em algum caminho
    void move_along_path(const vector<string>& path) {
        for (const auto& direction : path) {
            vector<string> moveResult = send_request(direction);
            usleep(10000);
            if (moveResult[0] != "true") {
                RCLCPP_ERROR(this->get_logger(), "Falha ao mover na direção: %s", direction.c_str());
                return;
            }
            path_.push_back(direction);
        }
    }

    // Peguei na net
    vector<string> reverse_path(const vector<string>& path) {
        map<string, string> opposite = {{"left", "right"}, {"right", "left"}, {"up", "down"}, {"down", "up"}};
        vector<string> reversed;
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            reversed.push_back(opposite[*it]);
        }
        return reversed;
    }

    // Idea GPT
    vector<pair<int, int>> reconstruct_path(map<pair<int, int>, pair<int, int>>& came_from, pair<int, int> current) { // Came from tem a posicao atual e de onde veio
        vector<pair<int, int>> total_path = {current};
        while (came_from.find(current) != came_from.end()) {
            current = came_from[current];
            total_path.push_back(current);
        }
        reverse(total_path.begin(), total_path.end());

        // Printar o caminho

        for (const auto& position : total_path) {
            RCLCPP_INFO(this->get_logger(), "Caminho: [%d, %d]", position.first, position.second);
        }

        return total_path;
    }


    // Obter vizinhos válidos
    vector<pair<int, int>> get_neighbors(pair<int, int> position) {
        vector<pair<int, int>> neighbors;
        vector<pair<int, int>> deltas = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // up down left right

        for (const auto& delta : deltas) {
            int nx = position.first + delta.first;
            int ny = position.second + delta.second;

            if (nx >= 0 && ny >= 0 && nx < 20 && ny < 20) {
                char cell = map_[nx][ny];
                if (cell == 'f' || cell == 't') {
                    neighbors.emplace_back(nx, ny);
                }
            }
        }
        return neighbors;
    }


    // DFs
    // TODO: logigc
    // Parecido com o dfs, eu também preciso salvar as celulas já visitasdas
    // Aqui preciso de uma pilha
    // Como no dfs eu uso uma fila, então lá eu vou explorando todos os visinhos de uma camada já que eles entraram primeiro na fila
    // Aqui eu preciso fazer que vá sendo explorado os vizinhos de um nó até que eu encontre o alvo ou que eu não tenha mais vizinhos para explorar ai eu volto até a posição que eu tinha um vizinho para explorar
    vector<pair<int, int>> find_path_dfs(pair<int, int> start, pair<int, int> goal) {
        stack<pair<int, int>> stack;
        map<pair<int, int>, pair<int, int>> came_from;
        set<pair<int, int>> visited;
        stack.push(start);

        while (!stack.empty()) {
            auto current = stack.top(); // Pego o topo da pilha
            stack.pop(); // Remover o topo da pilha como ele já foi explorado

            if (current == goal) {
                return reconstruct_path(came_from, current);
            }

            if (visited.find(current) != visited.end()) {
                continue;
            }
            visited.insert(current);

            for (const auto& neighbor : get_neighbors(current)) {
                if (visited.find(neighbor) == visited.end()) {
                    stack.push(neighbor);
                    came_from[neighbor] = current;
                }
            }
        }
        return {};
    }

    void follow_path(const vector<pair<int, int>>& path) {
        // Começar da posição atual
        for (size_t i = 1; i < path.size(); ++i) {
            auto [x_prev, y_prev] = path[i - 1]; // aqui eu pego a posição anterior 
            auto [x_curr, y_curr] = path[i]; // aqui eu pego a posição atual

            string direction;

            // Se a x diminui e y é o mesmo, então ele foi para cima
            if (x_curr == x_prev - 1 && y_curr == y_prev) {
                direction = "up";
            } else if (x_curr == x_prev + 1 && y_curr == y_prev) { // Se a x aumenta e y é o mesmo, então ele foi para baixo
                direction = "down";
            } else if (x_curr == x_prev && y_curr == y_prev - 1) { // Se a x é a mesma e y diminui, então ele foi para a esquerda
                direction = "left";
            } else if (x_curr == x_prev && y_curr == y_prev + 1) { // Se a x é a mesma e y aumenta, então ele foi para a direita
                direction = "right";
            } else {
                RCLCPP_ERROR(this->get_logger(), "Movimento inválido de (%d, %d) para (%d, %d)", x_prev, y_prev, x_curr, y_curr);
                return;
            }

            vector<string> moveResult = send_request(direction);
            if (moveResult[0] != "true") {
                RCLCPP_ERROR(this->get_logger(), "Falha ao mover na direção: %s", direction.c_str());
                return;
            }

        }
        RCLCPP_INFO(this->get_logger(), "Chegou ao alvo!");
    }


    void print_map() const {
        for (size_t i = 0; i < map_.size(); ++i) {
            for (size_t j = 0; j < map_[i].size(); ++j) {
                cout << map_[i][j] << " ";
            }
            cout << endl;
        }
    }

private:

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    vector<vector<char>> map_;
    vector<tuple<string, bool, array<int8_t, 2>, array<int8_t, 2>, string, string, string, string>> history_;
    default_random_engine gerador_;
    vector<string> path_;
    pair<int, int> target_position_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = make_shared<MoveMazeSolverClient>();

    node->bfs_nav();

    rclcpp::shutdown();
    return 0;
}
