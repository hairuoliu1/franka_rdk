#!/usr/bin/env python3
"""Set conservative Franka collision thresholds for one or more arm namespaces."""

import argparse
import sys
import time

import rclpy
from franka_msgs.srv import SetFullCollisionBehavior


DEFAULT_TORQUE_LOWER = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
DEFAULT_TORQUE_UPPER = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
DEFAULT_FORCE_LOWER = [34.0, 34.0, 34.0, 28.0, 28.0, 28.0]
DEFAULT_FORCE_UPPER = [45.0, 45.0, 45.0, 38.0, 38.0, 38.0]


def parse_threshold(values, expected_size, name):
    if len(values) != expected_size:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected_size} values, got {len(values)}"
        )
    return values


def normalize_namespace(namespace):
    return namespace.strip("/")


def resolve_service(node, namespace, explicit_service, timeout_sec):
    if explicit_service:
        return explicit_service

    namespace = normalize_namespace(namespace)
    deadline = time.monotonic() + timeout_sec

    while rclpy.ok() and time.monotonic() < deadline:
        names_and_types = node.get_service_names_and_types()
        candidates = []
        for name, types in names_and_types:
            if "franka_msgs/srv/SetFullCollisionBehavior" not in types:
                continue
            if not name.endswith("/set_full_collision_behavior"):
                continue
            if namespace:
                if name.startswith(f"/{namespace}/"):
                    candidates.append(name)
            elif name.count("/") == 2:
                candidates.append(name)

        if len(candidates) == 1:
            return candidates[0]
        if len(candidates) > 1:
            raise RuntimeError(
                "multiple matching services found for namespace "
                f"'{namespace}': {', '.join(candidates)}. Pass --service explicitly."
            )

        rclpy.spin_once(node, timeout_sec=0.1)

    raise RuntimeError(
        f"no set_full_collision_behavior service found for namespace '{namespace}'"
    )


def build_request(args):
    request = SetFullCollisionBehavior.Request()
    request.lower_torque_thresholds_acceleration = args.lower_torque
    request.upper_torque_thresholds_acceleration = args.upper_torque
    request.lower_torque_thresholds_nominal = args.lower_torque
    request.upper_torque_thresholds_nominal = args.upper_torque
    request.lower_force_thresholds_acceleration = args.lower_force
    request.upper_force_thresholds_acceleration = args.upper_force
    request.lower_force_thresholds_nominal = args.lower_force
    request.upper_force_thresholds_nominal = args.upper_force
    return request


def call_service(node, service_name, request, timeout_sec):
    client = node.create_client(SetFullCollisionBehavior, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"service not available: {service_name}")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise RuntimeError(f"service call timed out: {service_name}")

    response = future.result()
    if response is None:
        raise RuntimeError(f"service call failed: {service_name}")
    if not response.success:
        raise RuntimeError(f"service rejected request: {service_name}: {response.error}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Set Franka collision thresholds while keeping protection conservative. "
            "The default force thresholds are only moderately above Franka's examples."
        )
    )
    parser.add_argument(
        "namespaces",
        nargs="*",
        default=["left", "right"],
        help="arm namespaces to configure, for example: left right",
    )
    parser.add_argument(
        "--service",
        help="explicit SetFullCollisionBehavior service name; use for a single arm",
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--lower-torque",
        type=float,
        nargs=7,
        default=DEFAULT_TORQUE_LOWER,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
    )
    parser.add_argument(
        "--upper-torque",
        type=float,
        nargs=7,
        default=DEFAULT_TORQUE_UPPER,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
    )
    parser.add_argument(
        "--lower-force",
        type=float,
        nargs=6,
        default=DEFAULT_FORCE_LOWER,
        metavar=("X", "Y", "Z", "R", "P", "YAW"),
    )
    parser.add_argument(
        "--upper-force",
        type=float,
        nargs=6,
        default=DEFAULT_FORCE_UPPER,
        metavar=("X", "Y", "Z", "R", "P", "YAW"),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print resolved services and request values without calling services",
    )
    args = parser.parse_args()

    parse_threshold(args.lower_torque, 7, "lower-torque")
    parse_threshold(args.upper_torque, 7, "upper-torque")
    parse_threshold(args.lower_force, 6, "lower-force")
    parse_threshold(args.upper_force, 6, "upper-force")

    if args.service and len(args.namespaces) != 1:
        parser.error("--service can only be used with one namespace")

    return args


def main():
    args = parse_args()
    request = build_request(args)

    rclpy.init()
    node = rclpy.create_node("set_franka_collision_behavior")

    try:
        for namespace in args.namespaces:
            service_name = resolve_service(node, namespace, args.service, args.timeout)
            print(f"{namespace or '<root>'}: {service_name}")
            print(f"  lower_force: {list(request.lower_force_thresholds_nominal)}")
            print(f"  upper_force: {list(request.upper_force_thresholds_nominal)}")
            if not args.dry_run:
                call_service(node, service_name, request, args.timeout)
                print("  success")
    except Exception as exc:  # noqa: BLE001 - CLI should report any runtime failure.
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
