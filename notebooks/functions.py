"""Модуль муравьиного алгоритма
"""

from typing import Optional
from random import random
import pandas as pd

import numpy as np


def count_attraction(visited, unvisited, ph, rev_dist, p):
    """
    Вычисляет привлекательность данного пути для муравьев по формуле
    @:return: привлекательность пути
    """
    pheromone = ph[visited[-1], unvisited]  # количество феромона
    val = np.power(pheromone, p)
    attract = val * rev_dist[visited[-1], unvisited]
    attract /= np.sum(attract)
    return attract


def find_new_vertex(visited, unvisited, attract):
    """

    :param visited:
    :param unvisited:
    :param attract:
    :return:
    """
    rand_numb = random()  # число от 0 до 1
    for i in range(attract.size):
        if rand_numb < attract[i]:
            next_vertex = unvisited[i]
            break
        rand_numb -= attract[i]
    else:
        next_vertex = unvisited[-1]
    visited += (next_vertex,)
    return visited


def count_route_distance(dist, visited, available):
    route_dist = sum([dist[visited[i], visited[i + 1]]
                      for i in range(available)])
    return route_dist


def find_path(dist, start, end, ants=1, ages=1, rho=0.1, a=1, b=1,
              q=1, ph_min=0.01, ph_max=1, elite=0):
    """Муравьиный алгоритм.

    Поиск пути в графе с помощью муравьиного алгоритма

    Аргументы:
        dist: np.ndarray - матрица весов
        start: int - индекс начальной вершины
        end: int - индекс конечной вершины
        ants: int - количество муравьев в каждом поколении,
            по умолчанию 1
        ages: int - количество поколений, по умолчанию 1
        rho: float - скорость испарения феромона, по умолчанию 0.1
        a: float - (влияния феромона) стадность алгоритма
        b: float - (влияние близости вершины) жадность алгоритма
        q: float - количество откладываемого феромона,
            по умолчанию 1
        ph_min: float - минимальная концентрация феромона,
            по умолчанию 0.01
        ph_max: float - максимальная концентрация феромона,
            по умолчанию 1
        elite: int - количество элитных муравьев в каждом поколении,
            по умолчанию 0

    Возвращает кортеж с вершинами найденного пути и его длиной

    """
    num = dist.shape[0]
    # вычисляем обратные расстояния в
    rev_dist = np.zeros((num, num))
    for i in range(num):
        for j in range(num):
            if i != j:
                rev_dist[i, j] = 1 / dist[i, j] ** b
    # концентрации феромона
    ph = np.ones((num, num)) * ph_max

    best_route, best_dist = tuple(), np.inf

    # создаём список вершин
    vertexes = list(range(num))
    vertexes.remove(end)
    if start not in vertexes:
        vertexes.append(start)
    # число вершин доступных для перехода
    available = len(vertexes)
    for _ in range(ages):
        for _ in range(ants):
            visited = (start,)
            while len(visited) < available:
                unvisited = vertexes
                for item in unvisited:
                    if item in visited:
                        unvisited.remove(item)
                attract = count_attraction(visited, unvisited, ph, rev_dist, a)
                visited = find_new_vertex(visited, unvisited, attract)
    return best_route, best_dist
