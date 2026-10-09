#!/usr/bin/env python3
"""Small, offline CMA-ES implementation for bounded continuous PID search."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class CMAState:
    mean: list[float]
    covariance: list[list[float]]
    sigma: float
    path_c: list[float]
    path_sigma: list[float]
    generation: int
    best_fitness: float
    stagnant_generations: int


class BoundedCMAES:
    """CMA-ES in [0, 1]^n with rejection sampling at the boundaries.

    PID dimensions are normalized by their safe bounds, so differently scaled
    gains share one adaptive covariance model. Sigma and covariance are learned
    after every measured generation; there is no per-parameter fixed step.
    """

    def __init__(
        self,
        mean: list[float],
        population_size: int,
        *,
        sigma: float = 0.20,
        seed: int = 1,
        state: dict | None = None,
    ) -> None:
        if population_size < 4:
            raise ValueError("CMA-ES requires at least four evaluated candidates")
        self.n = len(mean)
        if self.n < 1:
            raise ValueError("CMA-ES requires at least one parameter")
        self.population_size = population_size
        self.mu = population_size // 2
        weights = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights = weights / np.sum(weights)
        self.mu_eff = 1.0 / np.sum(self.weights**2)
        n = self.n
        self.c_c = (4.0 + self.mu_eff / n) / (n + 4.0 + 2.0 * self.mu_eff / n)
        self.c_sigma = (self.mu_eff + 2.0) / (n + self.mu_eff + 5.0)
        self.c_1 = 2.0 / ((n + 1.3) ** 2 + self.mu_eff)
        self.c_mu = min(
            1.0 - self.c_1,
            2.0 * (self.mu_eff - 2.0 + 1.0 / self.mu_eff) / ((n + 2.0) ** 2 + self.mu_eff),
        )
        self.damping = (
            1.0
            + 2.0 * max(0.0, math.sqrt((self.mu_eff - 1.0) / (n + 1.0)) - 1.0)
            + self.c_sigma
        )
        self.chi_n = math.sqrt(n) * (1.0 - 1.0 / (4.0 * n) + 1.0 / (21.0 * n * n))
        self.rng = np.random.default_rng(seed)
        if state:
            restored = CMAState(**state)
            self.mean = np.asarray(restored.mean, dtype=float)
            self.covariance = np.asarray(restored.covariance, dtype=float)
            self.sigma = float(restored.sigma)
            self.path_c = np.asarray(restored.path_c, dtype=float)
            self.path_sigma = np.asarray(restored.path_sigma, dtype=float)
            self.generation = int(restored.generation)
            self.best_fitness = float(restored.best_fitness)
            self.stagnant_generations = int(restored.stagnant_generations)
        else:
            self.mean = np.clip(np.asarray(mean, dtype=float), 0.0, 1.0)
            self.covariance = np.eye(n)
            self.sigma = float(sigma)
            self.path_c = np.zeros(n)
            self.path_sigma = np.zeros(n)
            self.generation = 0
            self.best_fitness = math.inf
            self.stagnant_generations = 0

    def _eigensystem(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        covariance = 0.5 * (self.covariance + self.covariance.T)
        eigenvalues, basis = np.linalg.eigh(covariance)
        scales = np.sqrt(np.maximum(eigenvalues, 1.0e-12))
        inverse_root = basis @ np.diag(1.0 / scales) @ basis.T
        return basis, scales, inverse_root

    def ask(self, count: int | None = None) -> list[list[float]]:
        basis, scales, _ = self._eigensystem()
        samples: list[list[float]] = []
        for _ in range(self.population_size if count is None else count):
            # Rejection keeps the phenotype and the distribution update in the
            # same coordinate system. The fallback is only relevant near a
            # bound with an unusually large covariance.
            for _attempt in range(200):
                step = basis @ (scales * self.rng.normal(size=self.n))
                point = self.mean + self.sigma * step
                if np.all((0.0 <= point) & (point <= 1.0)):
                    break
            else:
                point = np.clip(point, 0.0, 1.0)
            samples.append(point.tolist())
        return samples

    def tell(self, points: list[list[float]], fitness: list[float]) -> None:
        if len(points) != self.population_size or len(fitness) != self.population_size:
            raise ValueError("tell() must receive exactly one complete CMA generation")
        order = np.argsort(np.asarray(fitness, dtype=float))
        selected = np.asarray(points, dtype=float)[order[: self.mu]]
        old_mean = self.mean.copy()
        self.mean = np.sum(selected * self.weights[:, None], axis=0)
        basis, scales, inverse_root = self._eigensystem()
        del basis, scales
        normalized_shift = (self.mean - old_mean) / max(self.sigma, 1.0e-12)
        self.path_sigma = (
            (1.0 - self.c_sigma) * self.path_sigma
            + math.sqrt(self.c_sigma * (2.0 - self.c_sigma) * self.mu_eff)
            * (inverse_root @ normalized_shift)
        )
        generation_factor = math.sqrt(
            max(1.0e-12, 1.0 - (1.0 - self.c_sigma) ** (2.0 * (self.generation + 1)))
        )
        h_sigma = float(np.linalg.norm(self.path_sigma)) / generation_factor / self.chi_n
        h_sigma = h_sigma < 1.4 + 2.0 / (self.n + 1.0)
        self.path_c = (
            (1.0 - self.c_c) * self.path_c
            + float(h_sigma)
            * math.sqrt(self.c_c * (2.0 - self.c_c) * self.mu_eff)
            * normalized_shift
        )
        selected_steps = (selected - old_mean) / max(self.sigma, 1.0e-12)
        rank_mu = sum(
            weight * np.outer(step, step)
            for weight, step in zip(self.weights, selected_steps)
        )
        correction = (1.0 - float(h_sigma)) * self.c_1 * self.c_c * (2.0 - self.c_c)
        self.covariance = (
            (1.0 - self.c_1 - self.c_mu + correction) * self.covariance
            + self.c_1 * np.outer(self.path_c, self.path_c)
            + self.c_mu * rank_mu
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.sigma *= math.exp(
            (self.c_sigma / self.damping)
            * (float(np.linalg.norm(self.path_sigma)) / self.chi_n - 1.0)
        )
        self.sigma = min(0.75, max(1.0e-5, self.sigma))
        generation_best = float(np.min(fitness))
        if generation_best < self.best_fitness - 1.0e-6:
            self.best_fitness = generation_best
            self.stagnant_generations = 0
        else:
            self.stagnant_generations += 1
        self.generation += 1

    def converged(self, sigma_tolerance: float, patience: int) -> bool:
        coordinate_scale = self.sigma * np.sqrt(np.maximum(np.diag(self.covariance), 0.0))
        return bool(np.max(coordinate_scale) <= sigma_tolerance or self.stagnant_generations >= patience)

    def state(self) -> dict:
        return CMAState(
            mean=self.mean.tolist(),
            covariance=self.covariance.tolist(),
            sigma=self.sigma,
            path_c=self.path_c.tolist(),
            path_sigma=self.path_sigma.tolist(),
            generation=self.generation,
            best_fitness=self.best_fitness,
            stagnant_generations=self.stagnant_generations,
        ).__dict__
