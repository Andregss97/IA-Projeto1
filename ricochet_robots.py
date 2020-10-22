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

    def __init__(self, size: int):
        self.size = size

    class Robot:

        def __init__(self, color: str, pos: tuple):
            self.color = color
            self.pos = pos

    class Wall:

        def __init__(self, wall_pos: tuple, wall_dir: str):
            self.wall_pos = wall_pos
            self.wall_dir = wall_dir


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

    def robotIsInPos(self, pos: tuple):
        for robot in self.Robots:
            if(robot.pos == pos):
                return True
        return False

    def wallIsInPos(self, pos: tuple):
        for wall in self.Walls:
            if(wall.wall_pos == pos):
                return True
        return False

    def getWallDir( self, pos: tuple):
        for wall in self.Walls:
            if(wall.wall_pos == pos):
                return wall.wall_dir
        return None


def parse_instance(filename: str) -> Board:
    """ Lê o ficheiro cujo caminho é passado como argumento e retorna
    uma instância da classe Board. """
    robots = []
    walls = []
    f = open(filename, "r")

    board = Board(int(f.readline()))

    for x in range(1, 5):
        line = f.readline()
        robots.append(board.Robot(line[0], (int(line[2]), int(line[4]))))

    board.init_robots(robots)

    line = f.readline()
    board.set_Objective(line[0], (int(line[2]), int(line[4])))

    board.Walls_nr(int(f.readline()))

    for x in range(0, board.nrWalls):
        line = f.readline()
        walls.append(board.Wall((int(line[0]), int(line[2])), line[4]))

    board.init_walls(walls)

    print(board.robot_position("Y"))
    Board.get_robots(board)
    print("Cor Obj:", board.obj_color, "Pos Obj:", board.obj_pos)
    print("Numero de walls:", board.nrWalls)
    Board.get_Walls(board)

    return board


class RicochetRobots(Problem):
    def __init__(self, board: Board):
        """ O construtor especifica o estado inicial. """
        self.initial = RRState(board)

    def actions(self, state: RRState):
        """ Retorna uma lista de ações que podem ser executadas a
        partir do estado passado como argumento. """
        #Fazer para os casos em que a parede está na posição ao lado
        actions = []
        board = state.board
        for robot in board.Robots:

#------------------------------------------------------------------------------------------#            
            if((robot.pos[0]-1)>0):
                if(board.robotIsInPos(((robot.pos[0]-1), robot.pos[1])) == False):
                    if(board.wallIsInPos(robot.pos)):
                        if(board.getWallDir(robot.pos) != 'u'):
                            actions.append((robot.color, 'u'))
                    
                    elif(board.wallIsInPos((robot.pos[0]-1, robot.pos[1]))):
                        if(board.getWallDir((robot.pos[0]-1, robot.pos[1])) != 'd'):
                            actions.append((robot.color, 'u'))

                    else:
                        actions.append((robot.color, 'u'))

#------------------------------------------------------------------------------------------#
            if((robot.pos[0]+1)<=board.size):
                if(board.robotIsInPos(((robot.pos[0]+1), robot.pos[1])) == False):
                    if(board.wallIsInPos(robot.pos)):
                        if(board.getWallDir(robot.pos) != 'd'):
                            actions.append((robot.color, 'd'))
                    
                    elif(board.wallIsInPos((robot.pos[0]+1, robot.pos[1]))):
                        if(board.getWallDir((robot.pos[0]+1, robot.pos[1])) != 'u'):
                            actions.append((robot.color, 'd'))

                    else:
                        actions.append((robot.color, 'd'))

#------------------------------------------------------------------------------------------#
            if((robot.pos[1]-1)>0):
                if(board.robotIsInPos(((robot.pos[0]), robot.pos[1]-1)) == False):
                    if(board.wallIsInPos(robot.pos)):
                        if(board.getWallDir(robot.pos) != 'l'):
                            actions.append((robot.color, 'l'))

                    elif(board.wallIsInPos((robot.pos[0], robot.pos[1]-1))):
                        if(board.getWallDir((robot.pos[0], robot.pos[1]-1)) != 'r'):
                            actions.append((robot.color, 'l'))

                    else:
                        actions.append((robot.color, 'l'))

#------------------------------------------------------------------------------------------#
            if((robot.pos[1]+1)<=board.size):
                if(board.robotIsInPos(((robot.pos[0]), robot.pos[1]+1)) == False):
                    if(board.wallIsInPos(robot.pos)):
                        if(board.getWallDir(robot.pos) != 'r'):
                            actions.append((robot.color, 'r'))
                    
                    elif(board.wallIsInPos((robot.pos[0], robot.pos[1]+1))):
                        if(board.getWallDir((robot.pos[0], robot.pos[1]+1)) != 'l'):
                            actions.append((robot.color, 'r'))

                    else:
                        actions.append((robot.color, 'r'))
        
        return actions

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
    
    board = parse_instance(sys.argv[1])
    problem = RicochetRobots(board)
    initial_state = RRState(board)
    print(problem.actions(initial_state))

