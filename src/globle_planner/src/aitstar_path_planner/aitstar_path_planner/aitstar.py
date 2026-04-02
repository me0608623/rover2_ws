#!/usr/bin/env python3
"""
AIT* (Adaptive Informed Trees) 路徑規劃演算法
============================================

AIT* 結合了 RRT* 的快速探索能力與 Informed RRT* 的啟發式採樣優化，
並透過批次處理與自適應採樣策略實現高效的最優路徑規劃。

主要特點：
- 啟發式採樣：在橢球區域內進行知情採樣
- 批次處理：批量新增頂點以提高效率
- 自適應策略：根據搜尋進度動態調整採樣區域
- 漸近最優：保證收斂至最優解

參考文獻：
Strub, M. P., & Gammell, J. D. (2020). 
"Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path Planning 
through Adaptive Heuristics"

作者: Claude
版本: 1.0.0
"""

from __future__ import annotations

import math
import heapq
import random
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set, Dict, Callable
from abc import ABC, abstractmethod
import numpy as np


# =============================================================================
# 資料結構定義
# =============================================================================

@dataclass
class State:
    """狀態空間中的一個點"""
    position: np.ndarray
    
    def __hash__(self):
        return hash(tuple(self.position))
    
    def __eq__(self, other):
        if not isinstance(other, State):
            return False
        return np.allclose(self.position, other.position)
    
    def distance_to(self, other: 'State') -> float:
        """計算到另一個狀態的歐氏距離"""
        return np.linalg.norm(self.position - other.position)


@dataclass
class Vertex:
    """圖中的頂點"""
    state: State
    parent: Optional['Vertex'] = None
    children: Set['Vertex'] = field(default_factory=set)
    cost_to_come: float = float('inf')  # g(v): 從起點到此頂點的代價
    cost_to_go: float = float('inf')     # h(v): 從此頂點到終點的啟發式估計
    
    def __hash__(self):
        return hash(self.state)
    
    def __eq__(self, other):
        if not isinstance(other, Vertex):
            return False
        return self.state == other.state
    
    def __lt__(self, other):
        """用於優先佇列排序"""
        return self.get_key() < other.get_key()
    
    def get_key(self) -> float:
        """取得頂點的排序鍵值 (f = g + h)"""
        return self.cost_to_come + self.cost_to_go


@dataclass
class Edge:
    """圖中的邊"""
    source: Vertex
    target: Vertex
    cost: float
    
    def __hash__(self):
        return hash((self.source, self.target))
    
    def __eq__(self, other):
        if not isinstance(other, Edge):
            return False
        return self.source == other.source and self.target == other.target
    
    def __lt__(self, other):
        """用於優先佇列排序"""
        return self.get_key() < other.get_key()
    
    def get_key(self) -> float:
        """取得邊的排序鍵值"""
        return (self.source.cost_to_come + self.cost + 
                self.target.cost_to_go)


# =============================================================================
# 碰撞檢測介面
# =============================================================================

class CollisionChecker(ABC):
    """碰撞檢測抽象介面"""
    
    @abstractmethod
    def is_state_valid(self, state: State) -> bool:
        """檢查狀態是否有效（無碰撞）"""
        pass
    
    @abstractmethod
    def is_edge_valid(self, state1: State, state2: State) -> bool:
        """檢查兩狀態之間的邊是否有效（無碰撞）"""
        pass

    @abstractmethod
    def get_obstacle_distance(self, state: State) -> float:
        """回傳節點到最近障礙物的距離"""
        pass


class SimpleCollisionChecker(CollisionChecker):
    """簡單的圓形障礙物碰撞檢測器"""
    
    def __init__(self, 
                 obstacles: List[Tuple[np.ndarray, float]],
                 bounds: Tuple[np.ndarray, np.ndarray],
                 robot_radius: float = 0.0):
        """
        初始化碰撞檢測器
        
        Args:
            obstacles: 障礙物列表 [(中心點, 半徑), ...]
            bounds: 空間邊界 (最小值, 最大值)
            robot_radius: 機器人半徑
        """
        self.obstacles = obstacles
        self.bounds_min, self.bounds_max = bounds
        self.robot_radius = robot_radius
    
    def is_state_valid(self, state: State) -> bool:
        """檢查狀態是否在邊界內且無碰撞"""
        pos = state.position
        
        # 檢查邊界
        if np.any(pos < self.bounds_min) or np.any(pos > self.bounds_max):
            return False
        
        # 檢查障礙物碰撞
        for obs_center, obs_radius in self.obstacles:
            dist = np.linalg.norm(pos - obs_center)
            if dist <= obs_radius + self.robot_radius:
                return False
        
        return True
    
    def is_edge_valid(self, state1: State, state2: State, 
                      resolution: float = 0.1) -> bool:
        """
        檢查邊是否有效（沿路徑採樣檢測）
        
        Args:
            state1: 起始狀態
            state2: 終止狀態
            resolution: 採樣解析度
        """
        if not self.is_state_valid(state1) or not self.is_state_valid(state2):
            return False
        
        direction = state2.position - state1.position
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return True
        
        n_steps = max(2, int(np.ceil(distance / resolution)))
        
        for i in range(1, n_steps):
            t = i / n_steps
            intermediate_pos = state1.position + t * direction
            intermediate_state = State(intermediate_pos)
            if not self.is_state_valid(intermediate_state):
                return False
        
        return True

    def get_obstacle_distance(self, state: State) -> float:
        """計算到最近障礙物邊緣的距離"""
        pos = state.position
        min_dist = float('inf')
        
        for obs_center, obs_radius in self.obstacles:
            dist = np.linalg.norm(pos - obs_center) - (obs_radius + self.robot_radius)
            if dist < min_dist:
                min_dist = dist
        
        return max(min_dist, 0.0)


# =============================================================================
# 採樣器
# =============================================================================

class Sampler:
    """狀態空間採樣器"""
    
    def __init__(self, 
                 bounds_min: np.ndarray, 
                 bounds_max: np.ndarray,
                 rng: Optional[np.random.Generator] = None):
        """
        初始化採樣器
        
        Args:
            bounds_min: 空間最小邊界
            bounds_max: 空間最大邊界
            rng: 隨機數生成器
        """
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max
        self.dimension = len(bounds_min)
        self.rng = rng if rng is not None else np.random.default_rng()
    
    def uniform_sample(self) -> State:
        """均勻隨機採樣"""
        position = self.rng.uniform(self.bounds_min, self.bounds_max)
        return State(position)
    
    def informed_sample(self, 
                       start: State, 
                       goal: State, 
                       c_best: float) -> State:
        """
        在橢球區域內進行知情採樣 (Informed Sampling)
        
        橢球定義：所有從 start 到 goal 經過的路徑長度 <= c_best 的點
        
        Args:
            start: 起始狀態
            goal: 目標狀態
            c_best: 當前最佳路徑代價
        """
        c_min = start.distance_to(goal)
        
        # 如果 c_best 無限大或接近 c_min，使用均勻採樣
        if c_best >= float('inf') or c_best <= c_min + 1e-6:
            return self.uniform_sample()
        
        # 計算橢球參數
        center = (start.position + goal.position) / 2.0
        
        # 主軸長度
        r1 = c_best / 2.0
        # 次軸長度
        ri = math.sqrt(c_best**2 - c_min**2) / 2.0
        
        # 計算旋轉矩陣
        a1 = (goal.position - start.position) / c_min
        rotation = self._rotation_to_world_frame(a1)
        
        # 在單位球內採樣
        x_ball = self._sample_unit_ball()
        
        # 縮放到橢球
        radii = np.array([r1] + [ri] * (self.dimension - 1))
        x_ellipse = np.diag(radii) @ x_ball
        
        # 旋轉並平移
        position = rotation @ x_ellipse + center
        
        # 確保在邊界內
        position = np.clip(position, self.bounds_min, self.bounds_max)
        
        return State(position)
    
    def _sample_unit_ball(self) -> np.ndarray:
        """在單位球內均勻採樣"""
        # 使用拒絕採樣
        while True:
            point = self.rng.uniform(-1, 1, self.dimension)
            if np.linalg.norm(point) <= 1.0:
                return point
    
    def _rotation_to_world_frame(self, a1: np.ndarray) -> np.ndarray:
        """
        計算從橢球坐標系到世界坐標系的旋轉矩陣
        
        Args:
            a1: 橢球主軸方向（世界坐標系）
        """
        # 使用 Rodrigues 旋轉公式
        e1 = np.zeros(self.dimension)
        e1[0] = 1.0
        
        # Gram-Schmidt 正交化建構完整的旋轉矩陣
        M = np.outer(a1, e1)
        U, _, Vt = np.linalg.svd(M)
        
        # 處理反射情況
        det = np.linalg.det(U @ Vt)
        diag = np.eye(self.dimension)
        diag[-1, -1] = det
        
        return U @ diag @ Vt


# =============================================================================
# AIT* 主演算法
# =============================================================================

class AITStar:
    """
    AIT* (Adaptive Informed Trees) 路徑規劃演算法
    
    AIT* 透過以下機制實現高效的漸近最優路徑規劃：
    1. 批次頂點新增：一次新增多個頂點以提高效率
    2. 知情採樣：在橢球區域內採樣以聚焦搜尋
    3. 自適應啟發式：動態更新啟發式估計
    4. 延遲碰撞檢測：優先檢查最有希望的邊
    """
    
    def __init__(self,
                 collision_checker: CollisionChecker,
                 bounds_min: np.ndarray,
                 bounds_max: np.ndarray,
                 batch_size: int = 100,
                 rewire_factor: float = 1.1,
                 goal_bias: float = 0.05,
                 max_iterations: int = 1000,
                 convergence_threshold: float = 1e-3,
                 w_obs: float = 0.0,
                 obs_threshold: float = 0.5,
                 seed: Optional[int] = None):
        """
        初始化 AIT* 規劃器
        
        Args:
            collision_checker: 碰撞檢測器
            bounds_min: 空間最小邊界
            bounds_max: 空間最大邊界
            batch_size: 每批次新增的頂點數
            rewire_factor: 重連半徑因子 (> 1.0)
            goal_bias: 目標偏向機率
            max_iterations: 最大迭代次數
            convergence_threshold: 收斂閾值
            seed: 隨機種子
        """
        self.collision_checker = collision_checker
        self.bounds_min = np.array(bounds_min)
        self.bounds_max = np.array(bounds_max)
        self.dimension = len(bounds_min)
        
        self.batch_size = batch_size
        self.rewire_factor = rewire_factor
        self.goal_bias = goal_bias
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.w_obs = w_obs
        self.obs_threshold = obs_threshold
        
        self.rng = np.random.default_rng(seed)
        self.sampler = Sampler(bounds_min, bounds_max, self.rng)
        
        # 規劃狀態
        self.vertices: Set[Vertex] = set()
        self.start_vertex: Optional[Vertex] = None
        self.goal_vertex: Optional[Vertex] = None
        self.solution_cost: float = float('inf')
        
        # 優先佇列
        self.edge_queue: List[Edge] = []
        self.vertex_queue: List[Vertex] = []
        
        # 統計資訊
        self.stats = {
            'iterations': 0,
            'vertices_sampled': 0,
            'edges_checked': 0,
            'collision_checks': 0,
            'solutions_found': 0
        }

        # KD-tree 用於加速鄰居查詢
        self._kd_tree = None
        self._vertex_list: List[Vertex] = []
        self._vertex_positions: np.ndarray = np.empty((0, self.dimension))
    
    def plan(self, 
             start: np.ndarray, 
             goal: np.ndarray,
             initial_solution: Optional[List[np.ndarray]] = None
             ) -> Tuple[Optional[List[np.ndarray]], float]:
        """
        執行路徑規劃
        
        Args:
            start: 起始位置
            goal: 目標位置
            initial_solution: 初始解（可選，用於熱啟動）
            
        Returns:
            (路徑點列表, 路徑代價) 或 (None, inf) 如果找不到路徑
        """
        self._initialize(start, goal)
        
        if initial_solution is not None:
            self._set_initial_solution(initial_solution)
        
        iteration = 0
        prev_cost = float('inf')
        no_improve_count = 0
        
        import time as _time
        _t0 = _time.time()

        while iteration < self.max_iterations:
            iteration += 1
            self.stats['iterations'] = iteration

            # 檢查是否需要新增新批次
            if self._should_add_batch():
                self._add_batch()

            # 處理頂點和邊佇列
            self._process_queues()

            # 每 50 次迭代輸出進度
            if iteration % 50 == 0:
                elapsed = _time.time() - _t0
                print(f"[AIT*] iter={iteration}/{self.max_iterations} "
                      f"vertices={len(self.vertices)} cost={self.solution_cost:.3f} "
                      f"elapsed={elapsed:.1f}s")

            # 檢查收斂：需要連續多個批次無明顯改進才停止
            if self.solution_cost < float('inf'):
                if prev_cost >= float('inf'):
                    improvement = 1.0  # 首次找到解，視為重大改進
                else:
                    improvement = (prev_cost - self.solution_cost) / prev_cost
                if improvement < self.convergence_threshold:
                    no_improve_count += 1
                else:
                    no_improve_count = 0
                # 至少運行 3 個批次，且連續 3 次無改進才退出
                if no_improve_count >= 3 and iteration > self.batch_size * 3:
                    break
                prev_cost = self.solution_cost

        elapsed = _time.time() - _t0
        print(f"[AIT*] finished: iter={iteration} vertices={len(self.vertices)} "
              f"cost={self.solution_cost:.3f} elapsed={elapsed:.1f}s")
        return self._extract_solution()

    def _calculate_edge_cost(self, s1: State, s2: State) -> float:
        """
        計算考慮障礙物勢能場的邊代價: c = dist + w * Penalty
        採用 Edge-based Adaptation 確保 h(v) 保持 Admissibility。
        """
        dist = s1.distance_to(s2)

        # 當 w_obs == 0 時，跳過昂貴的障礙物距離查詢
        if self.w_obs <= 0.0:
            return dist

        d_obs = self.collision_checker.get_obstacle_distance(s2)

        # 勢能場避障邏輯 (APF)
        if 0 < d_obs < self.obs_threshold:
            # 勢能場懲罰項: (1/d - 1/threshold)^2
            penalty = (1.0 / d_obs - 1.0 / self.obs_threshold) ** 2
            return dist + self.w_obs * penalty
        elif d_obs <= 1e-6:
            # 碰撞區域或極近距離給予無限大代價
            return float('inf')

        return dist
    
    def _initialize(self, start: np.ndarray, goal: np.ndarray):
        """初始化規劃問題"""
        self.vertices.clear()
        self.edge_queue.clear()
        self.vertex_queue.clear()
        self.solution_cost = float('inf')
        
        # 建立起點和終點頂點
        start_state = State(np.array(start))
        goal_state = State(np.array(goal))
        
        self.start_vertex = Vertex(start_state)
        self.start_vertex.cost_to_come = 0.0
        self.start_vertex.cost_to_go = start_state.distance_to(goal_state)
        
        self.goal_vertex = Vertex(goal_state)
        self.goal_vertex.cost_to_come = float('inf')
        self.goal_vertex.cost_to_go = 0.0
        
        # 驗證起點和終點
        if not self.collision_checker.is_state_valid(start_state):
            raise ValueError("起始位置處於碰撞狀態")
        if not self.collision_checker.is_state_valid(goal_state):
            raise ValueError("目標位置處於碰撞狀態")
        
        self.vertices.add(self.start_vertex)
        self.vertices.add(self.goal_vertex)

        # 初始化頂點佇列
        heapq.heappush(self.vertex_queue, self.start_vertex)

        # 初始化 KD-tree
        self._rebuild_kdtree()
    
    def _rebuild_kdtree(self):
        """重建 KD-tree 以加速鄰居查詢"""
        try:
            from scipy.spatial import cKDTree as _cKDTree
            self._vertex_list = list(self.vertices)
            if len(self._vertex_list) >= 2:
                self._vertex_positions = np.array(
                    [v.state.position for v in self._vertex_list]
                )
                self._kd_tree = _cKDTree(self._vertex_positions)
            else:
                self._kd_tree = None
        except ImportError:
            self._kd_tree = None

    def _set_initial_solution(self, path: List[np.ndarray]):
        """設定初始解（熱啟動）"""
        if len(path) < 2:
            return
        
        prev_vertex = self.start_vertex
        total_cost = 0.0
        
        for i in range(1, len(path) - 1):
            state = State(np.array(path[i]))
            vertex = Vertex(state)
            
            cost = self._calculate_edge_cost(prev_vertex.state, state)
            total_cost += cost
            
            vertex.cost_to_come = total_cost
            vertex.cost_to_go = state.distance_to(self.goal_vertex.state)
            vertex.parent = prev_vertex
            prev_vertex.children.add(vertex)
            
            self.vertices.add(vertex)
            prev_vertex = vertex
        
        # 連接到目標
        final_cost = self._calculate_edge_cost(prev_vertex.state, self.goal_vertex.state)
        total_cost += final_cost
        
        if self.collision_checker.is_edge_valid(prev_vertex.state, 
                                                 self.goal_vertex.state):
            self.goal_vertex.parent = prev_vertex
            self.goal_vertex.cost_to_come = total_cost
            prev_vertex.children.add(self.goal_vertex)
            self.solution_cost = total_cost
            self.stats['solutions_found'] += 1
    
    def _should_add_batch(self) -> bool:
        """判斷是否需要新增新的頂點批次"""
        # 當頂點佇列和邊佇列都為空時新增批次
        return len(self.vertex_queue) == 0 and len(self.edge_queue) == 0
    
    def _add_batch(self):
        """新增一批新的頂點"""
        new_vertices = []
        
        for _ in range(self.batch_size):
            # 採樣新狀態
            if self.rng.random() < self.goal_bias:
                state = self.goal_vertex.state
            elif self.solution_cost < float('inf'):
                # 知情採樣
                state = self.sampler.informed_sample(
                    self.start_vertex.state,
                    self.goal_vertex.state,
                    self.solution_cost
                )
            else:
                # 均勻採樣
                state = self.sampler.uniform_sample()
            
            self.stats['vertices_sampled'] += 1
            
            # 檢查狀態有效性
            if not self.collision_checker.is_state_valid(state):
                self.stats['collision_checks'] += 1
                continue
            
            # 建立新頂點
            vertex = Vertex(state)
            vertex.cost_to_come = float('inf')
            vertex.cost_to_go = state.distance_to(self.goal_vertex.state)
            
            # 檢查是否已存在相似頂點
            if self._find_similar_vertex(state) is not None:
                continue
            
            new_vertices.append(vertex)
            self.vertices.add(vertex)
        
        # 將新頂點加入佇列
        for vertex in new_vertices:
            heapq.heappush(self.vertex_queue, vertex)

        # 重建 KD-tree 以反映新頂點
        self._rebuild_kdtree()

        # 為現有頂點更新潛在邊
        self._update_edge_queue()
    
    def _find_similar_vertex(self, state: State,
                             threshold: float = 1e-3) -> Optional[Vertex]:
        """尋找與給定狀態相近的現有頂點"""
        if self._kd_tree is not None:
            idx = self._kd_tree.query_ball_point(state.position, r=threshold)
            if idx:
                return self._vertex_list[idx[0]]
            return None
        # Fallback：線性掃描
        for vertex in self.vertices:
            if vertex.state.distance_to(state) < threshold:
                return vertex
        return None
    
    def _update_edge_queue(self):
        """更新邊優先佇列（使用 KD-tree 批次查詢取代 O(N²) 雙重迴圈）"""
        self.edge_queue.clear()

        if self._kd_tree is None or len(self._vertex_list) < 2:
            # Fallback：原始雙重迴圈
            r = self._get_connection_radius()
            r_sq = r * r
            reachable = [v for v in self.vertices if v.cost_to_come < float('inf')]
            for vertex in reachable:
                vpos = vertex.state.position
                for neighbor in self.vertices:
                    if neighbor == vertex:
                        continue
                    if neighbor in vertex.children:
                        continue
                    diff = neighbor.state.position - vpos
                    dist_sq = float(diff @ diff)
                    if dist_sq > r_sq:
                        continue
                    dist = math.sqrt(dist_sq)
                    total_cost = vertex.cost_to_come + dist + neighbor.cost_to_go
                    if total_cost >= self.solution_cost:
                        continue
                    heapq.heappush(self.edge_queue, Edge(vertex, neighbor, dist))
            return

        r = self._get_connection_radius()
        reachable_idx = [i for i, v in enumerate(self._vertex_list)
                         if v.cost_to_come < float('inf')]
        if not reachable_idx:
            return

        reachable_positions = self._vertex_positions[reachable_idx]
        nbrs_list = self._kd_tree.query_ball_point(reachable_positions, r=r)

        for k, src_idx in enumerate(reachable_idx):
            src = self._vertex_list[src_idx]
            src_pos = self._vertex_positions[src_idx]
            child_states = {c.state for c in src.children}

            nbr_arr = np.array([i for i in nbrs_list[k] if i != src_idx])
            if len(nbr_arr) == 0:
                continue

            nbr_positions = self._vertex_positions[nbr_arr]
            diffs = nbr_positions - src_pos
            dists = np.sqrt((diffs ** 2).sum(axis=1))
            nbr_ctg = np.array([self._vertex_list[i].cost_to_go for i in nbr_arr])
            totals = src.cost_to_come + dists + nbr_ctg
            mask = totals < self.solution_cost

            for nbr_i, dist in zip(nbr_arr[mask], dists[mask]):
                nbr = self._vertex_list[nbr_i]
                if nbr.state in child_states:
                    continue
                heapq.heappush(self.edge_queue, Edge(src, nbr, float(dist)))
    
    def _get_informed_measure(self) -> float:
        """計算知情集 (prolate hyperspheroid) 的測度"""
        if self.solution_cost >= float('inf'):
            # 無解時使用整個搜尋空間的體積
            return float(np.prod(self.bounds_max - self.bounds_min))

        c_best = self.solution_cost
        c_min = self.start_vertex.state.distance_to(self.goal_vertex.state)
        if c_best <= c_min + 1e-6:
            return float(np.prod(self.bounds_max - self.bounds_min))

        d = self.dimension
        zeta_d = (math.pi ** (d / 2.0)) / math.gamma(d / 2.0 + 1)
        r1 = c_best / 2.0
        ri = math.sqrt(c_best**2 - c_min**2) / 2.0
        # 椭球体积 = zeta_d * r1 * ri^(d-1)
        return zeta_d * r1 * (ri ** (d - 1))

    def _get_connection_radius(self) -> float:
        """計算 RRG 連接半徑 (OMPL 公式)
        r = rewireFactor * [2*(1+1/d) * (mu/zeta_d) * (log(q)/q)]^(1/d)
        """
        n = len(self.vertices)
        if n < 2:
            return float('inf')

        d = self.dimension
        zeta_d = (math.pi ** (d / 2.0)) / math.gamma(d / 2.0 + 1)
        mu = self._get_informed_measure()

        r = self.rewire_factor * (
            2.0 * (1.0 + 1.0 / d) * (mu / zeta_d) * (math.log(n) / n)
        ) ** (1.0 / d)

        return min(r, np.linalg.norm(self.bounds_max - self.bounds_min))
    
    def _process_queues(self):
        """處理頂點和邊佇列"""
        # 限制每次處理的數量以保持響應性
        max_operations = self.batch_size
        operations = 0
        
        while operations < max_operations:
            operations += 1
            
            # 選擇處理頂點還是邊
            if self.edge_queue and self.vertex_queue:
                # 比較最佳頂點和最佳邊
                best_edge = self.edge_queue[0]
                best_vertex = self.vertex_queue[0]
                
                if best_vertex.get_key() <= best_edge.get_key():
                    self._expand_vertex()
                else:
                    self._process_edge()
            elif self.vertex_queue:
                self._expand_vertex()
            elif self.edge_queue:
                self._process_edge()
            else:
                break
    
    def _expand_vertex(self):
        """擴展頂點佇列中的最佳頂點"""
        if not self.vertex_queue:
            return

        vertex = heapq.heappop(self.vertex_queue)

        # 跳過不可達的頂點（cost_to_come 仍為 inf）
        if vertex.cost_to_come >= float('inf'):
            return

        # 如果頂點代價已超過當前最佳解，跳過（僅在有解時剪枝）
        if self.solution_cost < float('inf') and vertex.get_key() >= self.solution_cost:
            return
        
        # 尋找可連接的鄰居（使用 KD-tree 取代線性掃描）
        r = self._get_connection_radius()

        if self._kd_tree is not None:
            indices = self._kd_tree.query_ball_point(vertex.state.position, r=r)
            neighbors = [self._vertex_list[i] for i in indices
                         if self._vertex_list[i] != vertex]
        else:
            neighbors = [v for v in self.vertices if v != vertex
                         and vertex.state.distance_to(v.state) <= r]

        for neighbor in neighbors:
            dist = vertex.state.distance_to(neighbor.state)
            if vertex.cost_to_come + dist + neighbor.cost_to_go < self.solution_cost:
                heapq.heappush(self.edge_queue, Edge(vertex, neighbor, dist))
    
    def _process_edge(self):
        """處理邊佇列中的最佳邊"""
        if not self.edge_queue:
            return
        
        edge = heapq.heappop(self.edge_queue)
        self.stats['edges_checked'] += 1
        
        # 檢查邊是否仍有改進潛力
        if edge.get_key() >= self.solution_cost:
            return
        
        source, target = edge.source, edge.target
        new_cost = source.cost_to_come + edge.cost
        
        # 如果沒有改進，跳過
        if new_cost >= target.cost_to_come:
            return
        
        # 執行碰撞檢測
        self.stats['collision_checks'] += 1
        if not self.collision_checker.is_edge_valid(source.state, target.state):
            return
        
        # 更新連接
        if target.parent is not None:
            target.parent.children.discard(target)

        target.parent = source
        target.cost_to_come = new_cost
        source.children.add(target)

        # 如果連接到目標，更新最佳解
        if target == self.goal_vertex:
            if new_cost < self.solution_cost:
                self.solution_cost = new_cost
                self.stats['solutions_found'] += 1

        # 將更新後的頂點重新加入頂點佇列以便擴展出新邊
        heapq.heappush(self.vertex_queue, target)

        # 傳播代價更新到子節點
        self._propagate_cost_update(target)
    
    def _propagate_cost_update(self, vertex: Vertex):
        """遞迴傳播代價更新到子節點"""
        for child in vertex.children:
            edge_dist = self._calculate_edge_cost(vertex.state, child.state)
            new_cost = vertex.cost_to_come + edge_dist
            if new_cost < child.cost_to_come:
                child.cost_to_come = new_cost
                
                if child == self.goal_vertex:
                    if new_cost < self.solution_cost:
                        self.solution_cost = new_cost
                
                self._propagate_cost_update(child)
    
    def _extract_solution(self) -> Tuple[Optional[List[np.ndarray]], float]:
        """提取最終路徑"""
        if self.goal_vertex.parent is None:
            return None, float('inf')
        
        path = []
        current = self.goal_vertex
        
        while current is not None:
            path.append(current.state.position.copy())
            current = current.parent
        
        path.reverse()
        return path, self.solution_cost
    
    def get_tree(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """取得搜尋樹的所有邊（用於視覺化）"""
        edges = []
        for vertex in self.vertices:
            if vertex.parent is not None:
                edges.append((
                    vertex.parent.state.position.copy(),
                    vertex.state.position.copy()
                ))
        return edges
    
    def get_statistics(self) -> Dict:
        """取得規劃統計資訊"""
        return {
            **self.stats,
            'total_vertices': len(self.vertices),
            'solution_cost': self.solution_cost
        }


# =============================================================================
# 路徑平滑與後處理
# =============================================================================

class PathSmoother:
    """路徑平滑處理器"""
    
    def __init__(self, collision_checker: CollisionChecker):
        """
        初始化路徑平滑器
        
        Args:
            collision_checker: 碰撞檢測器
        """
        self.collision_checker = collision_checker
    
    def shortcut(self, path: List[np.ndarray], 
                 max_iterations: int = 100) -> List[np.ndarray]:
        """
        路徑捷徑優化：嘗試跳過中間點以縮短路徑
        
        Args:
            path: 原始路徑
            max_iterations: 最大迭代次數
        """
        if len(path) < 3:
            return path
        
        smoothed = [p.copy() for p in path]
        
        for _ in range(max_iterations):
            if len(smoothed) < 3:
                break
            
            # 隨機選擇兩個非相鄰點
            i = random.randint(0, len(smoothed) - 3)
            j = random.randint(i + 2, len(smoothed) - 1)
            
            # 嘗試直接連接
            state_i = State(smoothed[i])
            state_j = State(smoothed[j])
            
            if self.collision_checker.is_edge_valid(state_i, state_j):
                # 移除中間點
                smoothed = smoothed[:i+1] + smoothed[j:]
        
        return smoothed

    def smooth(self, path: List[np.ndarray],
               weight_smooth: float = 0.3,
               weight_data: float = 0.5,
               max_iterations: int = 200,
               tolerance: float = 1e-4) -> List[np.ndarray]:
        """
        梯度下降路徑平滑：最小化路徑曲率同時保持接近原始路徑
        """
        if len(path) < 3:
            return path

        smoothed = np.array([p.copy() for p in path], dtype=float)
        original = smoothed.copy()

        for _ in range(max_iterations):
            max_change = 0.0
            for i in range(1, len(smoothed) - 1):
                for d in range(smoothed.shape[1]):
                    old = smoothed[i][d]
                    smoothed[i][d] += (
                        weight_data * (original[i][d] - smoothed[i][d]) +
                        weight_smooth * (smoothed[i-1][d] + smoothed[i+1][d] - 2.0 * smoothed[i][d])
                    )
                    max_change = max(max_change, abs(smoothed[i][d] - old))
            if max_change < tolerance:
                break

        # 驗證平滑後路徑無碰撞
        result = [smoothed[0].copy()]
        for i in range(1, len(smoothed)):
            s1 = State(result[-1])
            s2 = State(smoothed[i])
            if self.collision_checker.is_edge_valid(s1, s2):
                result.append(smoothed[i].copy())
            else:
                result.append(path[i].copy())
        return result

    def interpolate(self, path: List[np.ndarray], 
                    resolution: float = 0.1) -> List[np.ndarray]:
        """
        路徑插值：在路徑點之間插入中間點
        
        Args:
            path: 原始路徑
            resolution: 插值解析度
        """
        if len(path) < 2:
            return path
        
        interpolated = [path[0].copy()]
        
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            
            direction = end - start
            distance = np.linalg.norm(direction)
            
            if distance < 1e-6:
                continue
            
            n_segments = max(1, int(np.ceil(distance / resolution)))
            
            for j in range(1, n_segments + 1):
                t = j / n_segments
                point = start + t * direction
                interpolated.append(point.copy())
        
        return interpolated


# =============================================================================
# 測試與範例
# =============================================================================

def create_test_environment():
    """建立測試環境"""
    # 空間邊界
    bounds_min = np.array([0.0, 0.0])
    bounds_max = np.array([10.0, 10.0])
    
    # 障礙物 (圓形)
    obstacles = [
        (np.array([3.0, 3.0]), 1.0),
        (np.array([5.0, 5.0]), 1.5),
        (np.array([7.0, 3.0]), 1.0),
        (np.array([3.0, 7.0]), 0.8),
        (np.array([7.0, 7.0]), 1.2),
    ]
    
    collision_checker = SimpleCollisionChecker(
        obstacles=obstacles,
        bounds=(bounds_min, bounds_max),
        robot_radius=0.2
    )
    
    return collision_checker, bounds_min, bounds_max, obstacles


def main():
    """主測試函式"""
    print("=" * 60)
    print("AIT* 路徑規劃演算法測試")
    print("=" * 60)
    
    # 建立測試環境
    collision_checker, bounds_min, bounds_max, obstacles = create_test_environment()
    
    # 建立規劃器
    planner = AITStar(
        collision_checker=collision_checker,
        bounds_min=bounds_min,
        bounds_max=bounds_max,
        batch_size=50,
        rewire_factor=1.1,
        goal_bias=0.05,
        max_iterations=500,
        seed=42
    )
    
    # 定義起點和終點
    start = np.array([1.0, 1.0])
    goal = np.array([9.0, 9.0])
    
    print(f"\n起點: {start}")
    print(f"終點: {goal}")
    print(f"障礙物數量: {len(obstacles)}")
    
    # 執行規劃
    import time
    start_time = time.time()
    path, cost = planner.plan(start, goal)
    planning_time = time.time() - start_time
    
    # 輸出結果
    print(f"\n規劃時間: {planning_time:.3f} 秒")
    stats = planner.get_statistics()
    print(f"迭代次數: {stats['iterations']}")
    print(f"頂點數量: {stats['total_vertices']}")
    print(f"採樣頂點: {stats['vertices_sampled']}")
    print(f"邊檢查數: {stats['edges_checked']}")
    print(f"碰撞檢查: {stats['collision_checks']}")
    print(f"找到解數: {stats['solutions_found']}")
    
    if path is not None:
        print(f"\n找到路徑!")
        print(f"路徑代價: {cost:.3f}")
        print(f"路徑點數: {len(path)}")
        
        # 路徑平滑
        smoother = PathSmoother(collision_checker)
        smoothed_path = smoother.shortcut(path, max_iterations=50)
        smoothed_cost = sum(
            np.linalg.norm(smoothed_path[i+1] - smoothed_path[i]) 
            for i in range(len(smoothed_path) - 1)
        )
        
        print(f"\n平滑後路徑:")
        print(f"路徑代價: {smoothed_cost:.3f}")
        print(f"路徑點數: {len(smoothed_path)}")
    else:
        print("\n未找到有效路徑!")
    
    print("\n" + "=" * 60)
    return path, cost


if __name__ == "__main__":
    main()