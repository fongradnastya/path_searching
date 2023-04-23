"""Модуль муравьиного алгоритма
"""

from typing import Optional
from random import random

import numpy as np


def count_attraction(visited, unvisited, ph, rev_dist, a):
    """
    Вычисляет привлекательность данного пути для муравьев
    :return:
    """
    attract = (np.power(ph[visited[-1], unvisited], a)
               * rev_dist[visited[-1], unvisited])
    attract /= np.sum(attract)
    return attract


def find_path(dist, start, end, *, ants=1, ages=1, rho=0.1, a=1, b=1,
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
        a: float - влияния феромона, по умолчанию 1
        b: float - влияние близости вершины, по умолчанию 1
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
    return best_route, best_dist
