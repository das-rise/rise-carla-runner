import logging
import math
from collections import Counter
from typing import Any, Dict, Iterable, Optional, Sequence, Set


class SceneObjectEntropyTracker:
    """Track scene object-class occurrences over a simulation run."""

    _STATIC_PROP_KEYWORDS = {
        "tree": {"tree", "oak", "pine", "birch", "palm", "maple"},
        "house": {"house", "home", "building", "hut", "garage"},
        "barrier": {"barrier", "guardrail", "fence", "wall"},
        "sign": {"sign", "trafficwarning", "warning", "billboard"},
        "street_furniture": {
            "bench",
            "trash",
            "trashcan",
            "lamp",
            "light",
            "pole",
            "mailbox",
        },
    }
    _ENVIRONMENT_OBJECT_TYPE_CLASSIFICATIONS = {
        "Bicycle": "vehicle",
        "Buildings": "building",
        "Bus": "vehicle",
        "Car": "vehicle",
        "Dynamic": "environment_dynamic",
        "Fences": "static_prop_barrier",
        "GuardRail": "static_prop_barrier",
        "Motorcycle": "vehicle",
        "Other": "environment_other",
        "Pedestrians": "walker",
        "Poles": "static_prop_street_furniture",
        "Static": "environment_static",
        "TrafficLight": "traffic_light",
        "TrafficSigns": "traffic_sign",
        "Train": "vehicle",
        "Truck": "vehicle",
        "Vegetation": "vegetation",
        "Walls": "static_prop_barrier",
    }

    def __init__(
        self,
        world: Any,
        exclude_type_prefixes: Optional[Sequence[str]] = None,
        exclude_type_ids: Optional[Sequence[str]] = None,
        debug: bool = False,
    ) -> None:
        self._world = world
        self._exclude_type_prefixes = tuple(
            exclude_type_prefixes or ("sensor.", "controller.")
        )
        self._exclude_type_ids = set(exclude_type_ids or ("spectator",))
        self._debug = debug
        self._active = False
        self._class_occurrences: Counter[str] = Counter()
        self._debug_actor_inventory: Counter[str] = Counter()
        self._debug_environment_object_inventory: Counter[str] = Counter()
        self._sample_count = 0
        self._classes_present_in_all_samples: Optional[Set[str]] = None

    def start(self) -> None:
        """Start a new counting window for a simulation run."""
        self._class_occurrences.clear()
        self._debug_actor_inventory.clear()
        self._debug_environment_object_inventory.clear()
        self._sample_count = 0
        self._classes_present_in_all_samples = None
        self._active = True

    def stop(self) -> None:
        """Stop sampling without discarding the current aggregate."""
        self._active = False

    @classmethod
    def classify_type_id(cls, actor_type_id: str) -> str:
        """Normalize CARLA actor type ids into coarse scene object classes."""
        if actor_type_id.startswith("vehicle."):
            return "vehicle"
        if actor_type_id.startswith("walker."):
            return "walker"
        if actor_type_id == "traffic.traffic_light" or "traffic_light" in actor_type_id:
            return "traffic_light"
        if actor_type_id.startswith("traffic."):
            return "traffic_sign"
        if actor_type_id.startswith("sensor."):
            return "sensor"
        if actor_type_id.startswith("static.prop."):
            return cls._classify_static_prop_type_id(actor_type_id)
        return "other"

    @classmethod
    def _classify_static_prop_type_id(cls, actor_type_id: str) -> str:
        """Split static props into more informative categories when possible."""
        prop_suffix = actor_type_id[len("static.prop.") :]
        classified = cls._classify_named_static_object(
            prop_suffix, fallback_to_first_token=True
        )
        return classified or "static_prop"

    @classmethod
    def _classify_named_static_object(
        cls, object_name: str, fallback_to_first_token: bool = False
    ) -> Optional[str]:
        """Classify a named static object using tokens embedded in its identifier."""
        if not object_name:
            return None

        tokens = []
        for part in object_name.split("."):
            tokens.extend(token for token in part.lower().split("_") if token)

        for category, keywords in cls._STATIC_PROP_KEYWORDS.items():
            if any(
                token == keyword or token.startswith(keyword) or token.endswith(keyword)
                for token in tokens
                for keyword in keywords
            ):
                return f"static_prop_{category}"

        if fallback_to_first_token and tokens:
            return f"static_prop_{tokens[0]}"
        return None

    @staticmethod
    def _get_environment_object_type_name(environment_object_type: Any) -> str:
        """Normalize a CityObjectLabel-like value to a stable string name."""
        if environment_object_type is None:
            return ""
        return getattr(
            environment_object_type, "name", str(environment_object_type).split(".")[-1]
        )

    @classmethod
    def classify_environment_object(cls, environment_object: Any) -> str:
        """Normalize a CARLA environment object into a scene object class."""
        object_name = getattr(environment_object, "name", "") or ""
        named_classification = cls._classify_named_static_object(object_name)
        object_type_name = cls._get_environment_object_type_name(
            getattr(environment_object, "type", None)
        )

        if object_type_name in {
            "Buildings",
            "Vegetation",
            "Fences",
            "GuardRail",
            "Poles",
            "Walls",
        }:
            if named_classification is not None:
                return named_classification

        if object_type_name in cls._ENVIRONMENT_OBJECT_TYPE_CLASSIFICATIONS:
            return cls._ENVIRONMENT_OBJECT_TYPE_CLASSIFICATIONS[object_type_name]

        if named_classification is not None:
            return named_classification

        return "environment_object"

    def _should_exclude(self, actor_type_id: str) -> bool:
        return actor_type_id in self._exclude_type_ids or actor_type_id.startswith(
            self._exclude_type_prefixes
        )

    def _iter_actor_type_ids(self) -> Iterable[str]:
        for actor in self._world.get_actors():
            actor_type_id = getattr(actor, "type_id", None)
            if not actor_type_id:
                continue
            yield actor_type_id

    def _iter_environment_objects(self) -> Iterable[Any]:
        get_environment_objects = getattr(self._world, "get_environment_objects", None)
        if get_environment_objects is None:
            return

        for environment_object in get_environment_objects():
            yield environment_object

    @classmethod
    def _format_environment_object_debug_entry(
        cls, environment_object: Any, classification: Optional[str] = None
    ) -> str:
        """Create a readable debug string for one environment object."""
        object_type_name = cls._get_environment_object_type_name(
            getattr(environment_object, "type", None)
        )
        object_name = getattr(environment_object, "name", "") or "<unnamed>"
        if classification is None:
            classification = cls.classify_environment_object(environment_object)
        return f"{object_type_name}:{object_name} -> {classification}"

    def sample(self) -> Dict[str, int]:
        """Take one scene sample and accumulate object-class counts."""
        if not self._active:
            return {}

        sample_counts: Counter[str] = Counter()
        for actor_type_id in self._iter_actor_type_ids():
            if self._should_exclude(actor_type_id):
                if self._debug:
                    self._debug_actor_inventory[f"{actor_type_id} -> excluded"] += 1
                continue

            actor_classification = self.classify_type_id(actor_type_id)
            sample_counts[actor_classification] += 1
            if self._debug:
                self._debug_actor_inventory[
                    f"{actor_type_id} -> {actor_classification}"
                ] += 1

        for environment_object in self._iter_environment_objects():
            environment_object_classification = self.classify_environment_object(
                environment_object
            )
            sample_counts[environment_object_classification] += 1
            if self._debug:
                self._debug_environment_object_inventory[
                    self._format_environment_object_debug_entry(
                        environment_object,
                        environment_object_classification,
                    )
                ] += 1

        self._class_occurrences.update(sample_counts)
        self._sample_count += 1

        sampled_classes = set(sample_counts)
        if self._classes_present_in_all_samples is None:
            self._classes_present_in_all_samples = sampled_classes
        else:
            self._classes_present_in_all_samples &= sampled_classes

        return dict(sample_counts)

    def debug_inventory_summary(self) -> Dict[str, Dict[str, int]]:
        """Return the raw scene inventory observed while debug mode was enabled."""
        return {
            "actor_inventory": dict(sorted(self._debug_actor_inventory.items())),
            "environment_object_inventory": dict(
                sorted(self._debug_environment_object_inventory.items())
            ),
        }

    def evaluate(self) -> float:
        """Return Shannon entropy in bits for the aggregated class distribution."""
        total_observations = sum(self._class_occurrences.values())
        if total_observations == 0:
            return 0.0

        entropy = 0.0
        for class_count in self._class_occurrences.values():
            probability = class_count / total_observations
            entropy -= probability * math.log2(probability)
        return entropy

    def summary(self) -> Dict[str, Any]:
        """Return a stable, log-friendly summary of the tracked run."""
        total_observations = sum(self._class_occurrences.values())
        class_counts = dict(sorted(self._class_occurrences.items()))
        if total_observations == 0:
            class_probabilities = {}
        else:
            class_probabilities = {
                class_name: count / total_observations
                for class_name, count in class_counts.items()
            }

        return {
            "sample_count": self._sample_count,
            "total_observations": total_observations,
            "class_counts": class_counts,
            "class_probabilities": class_probabilities,
            "classes_present_in_all_samples": sorted(
                self._classes_present_in_all_samples or []
            ),
            "entropy_bits": self.evaluate(),
        }

    def log_summary(self, logger: Optional[logging.Logger] = None) -> None:
        """Emit the final summary through the configured logger."""
        logger = logger or logging.getLogger()
        summary = self.summary()
        logger.info(
            "Scene object entropy: %.6f bits across %d samples and %d observations. "
            "Class counts=%s. Classes present in every sample=%s",
            summary["entropy_bits"],
            summary["sample_count"],
            summary["total_observations"],
            summary["class_counts"],
            summary["classes_present_in_all_samples"],
        )
        if self._debug:
            debug_inventory = self.debug_inventory_summary()
            logger.info(
                "Scene object entropy debug actor inventory: %s",
                debug_inventory["actor_inventory"],
            )
            logger.info(
                "Scene object entropy debug environment-object inventory: %s",
                debug_inventory["environment_object_inventory"],
            )
