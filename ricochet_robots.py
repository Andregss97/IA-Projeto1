# ricochet_robots.py: Template para implementação do 1º projeto de Inteligência Artificial 2020/2021.
# Devem alterar as classes e funções neste ficheiro de acordo com as instruções do enunciado.
# Além das funções e classes já definidas, podem acrescentar outras que considerem pertinentes.

# Grupo 28:
# 84699 André Santos
# 84722 Gonçalo Castilho

from search import Problem, Node, astar_search, breadth_first_tree_search, \
    depth_first_tree_search, greedy_search
import sys


class RRState:
    state_id = 0

    def __init__(self, board):
        self.board = board
        self.id = RRState.state_id
        RRState.state_id += 1

    def __lt__(self, other):
        """ Este método é utilizado em caso de empate na gestão da lista
        de abertos nas procuras informadas. """
        return self.id < other.id


class Board:
    """ Representacao interna de um tabuleiro de Ricochet Robots. """

    class Robot:

        def __init__(self, color: str, pos: tuple):
            self.color = color
            self.pos = pos

    class Wall:

        def __init__(self, wall_pos: tuple, wall_dir: str):
            self.wall_pos = wall_pos
            self.wall_dir = wall_dir


    def __init__(self, size: int):
        self.size = size

    def Walls_nr(self, nrWalls: int):
        self.nrWalls = nrWalls
    
    def init_robots(self, Robots: Robot):
        self.Robots = Robots

    def init_walls(self, walls: Wall):
        self.Walls = walls

    def set_Objective(self, color: str, pos: int):
        self.obj_color = color
        self.obj_pos = pos

    def robot_position(self, robot: str):
        """ Devolve a posição atual do robô passado como argumento. """
        for Robot in self.Robots:
            attr = getattr(Robot, "color")
            if(attr == robot):
                print(attr)
                return (Robot.pos)

    def get_robots(self):
        for robot in self.Robots:
            print("cor:", robot.color, "pos:", robot.pos)

    def get_Walls(self):
        for wall in self.Walls:
            print("pos:", wall.wall_pos, "dir:", wall.wall_dir)


def parse_instance(filename: str) -> Board:
    """ Lê o ficheiro cujo caminho é passado como argumento e retorna
    uma instância da classe Board. """
    robots = []
    walls = []
    f = open(filename, "r")

    board = Board(f.readline())

    for x in range(1, 5):
        line = f.readline()
        robots.append(Board.Robot(line[0], (int(line[2]), int(line[4]))))

    Board.init_robots(board, robots)

    line = f.readline()
    Board.set_Objective(board, line[0], (int(line[2]), int(line[4])))

    Board.Walls_nr(board, int(f.readline()))

    for x in range(0, board.nrWalls):
        line = f.readline()
        walls.append(Board.Wall((int(line[0]), int(line[2])), line[4]))

    Board.init_walls(board, walls)

    print(Board.robot_position(board, "Y"))
    Board.get_robots(board)
    print("Cor Obj:", board.obj_color, "Pos Obj:", board.obj_pos)
    print("Numero de walls:", board.nrWalls)
    Board.get_Walls(board)

    return board


class RicochetRobots(Problem):
    def __init__(self, board: Board):
        """ O construtor especifica o estado inicial. """
        # TODO: self.initial = ...
        pass

    def actions(self, state: RRState):
        """ Retorna uma lista de ações que podem ser executadas a
        partir do estado passado como argumento. """
        # TODO
        pass

    def result(self, state: RRState, action):
        """ Retorna o estado resultante de executar a 'action' sobre
        'state' passado como argumento. A ação retornada deve ser uma
        das presentes na lista obtida pela execução de
        self.actions(state). """
        # TODO
        pass

    def goal_test(self, state: RRState):
        """ Retorna True se e só se o estado passado como argumento é
        um estado objetivo. Deve verificar se o alvo e o robô da
        mesma cor ocupam a mesma célula no tabuleiro. """
        # TODO
        pass

    def h(self, node: Node):
        """ Função heuristica utilizada para a procura A*. """
        # TODO
        pass


if __name__ == "__main__":
    # TODO:
    # Ler o ficheiro de input de sys.argv[1],
    # Usar uma técnica de procura para resolver a instância,
    # Retirar a solução a partir do nó resultante,
    # Imprimir para o standard output no formato indicado.
    
	parse_instance(sys.argv[1])