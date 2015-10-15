#!/usr/bin/env python
import rospy
import sys
import argparse
import yaml
import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF


def get_joint_with(joints, attr, value):
    result = [joint for joint in joints if getattr(joint, attr) == value]
    return result[0] if result else None


def get_chain(joints, start_joint, end_joint):
    reverse_chain = [end_joint]
    joint = end_joint
    while (joint != start_joint):
        joint = get_joint_with(joints, 'child', joint.parent)
        if joint is None:
            print('No connection between %s and %s', start_joint.name, end_joint.name)
            return None
        else:
            reverse_chain.append(joint)
    return list(reversed(reverse_chain))


def input_variables_definition(chain):
    input_vars = []
    i = 0
    for joint in chain:
        input_vars.append({(joint.name + '_var'): {'input-var': i}})
        i += 1
    return input_vars


def joint_transform_definition(chain):
    joint_transforms = []
    get_joint_transform = {
        'fixed': lambda joint: fixed_joint_transform(joint),
        'prismatic': lambda joint: prismatic_joint_transform(joint),
        'revolute': lambda joint: revolute_joint_transform(joint),
        'continuous': lambda joint: revolute_joint_transform(joint)
    }
    for joint in chain:
        joint_transforms.append(
            get_joint_transform[joint.type](joint)
        )
    return joint_transforms

def prismatic_joint_transform(joint):
    print 'asda'
    translation = []
    for i in range(0,3):
        if joint.axis[i] == 1:
            print 'dfg'
            translation.append(
                {'double-add': [joint.origin.xyz[i], joint.name + '_var']}
            )
        else:
            print 'jhg'
            translation.append(joint.origin.xyz[i])
    transform = {(joint.name + '_frame'): {'frame': [
                    {'axis-angle': [{'vector3': joint.axis}, 0]},
                    {'vector3': translation}
                ]}}
    return transform


def revolute_joint_transform(joint):
    transform = {(joint.name + '_frame'): {'frame': [
                    {'axis-angle': [{'vector3': joint.axis}, joint.name + '_var']},
                    {'vector3': joint.origin.xyz}
                ]}}
    return transform


def fixed_joint_transform(joint):
    transform = {(joint.name + '_frame'): {'frame': [
                    {'axis-angle': [{'vector3': joint.axis}, 0]},
                    {'vector3': joint.origin.xyz}
                ]}}
    return transform


def fk_definition(chain):
    fk_joint_names = []
    for joint in chain:
        fk_joint_names.append(joint.name + '_frame')
    return {'pr2_fk': {'frame-mul': fk_joint_names}}


def get_scope_yaml(robot, start_link_name, end_link_name):
    start_joint = get_joint_with(robot.joints, 'parent', start_link_name)
    end_joint = get_joint_with(robot.joints, 'child', end_link_name)
    chain = get_chain(robot.joints, start_joint, end_joint)

    input_vars = input_variables_definition(chain)
    joint_transforms = joint_transform_definition(chain)
    fk_def = fk_definition(chain)

    return input_vars + joint_transforms + [fk_def]


def main():
    parser = argparse.ArgumentParser(usage='Get the giskard yaml description for the specified chain in the urdf.')
    parser.add_argument('start', type=str, default=None, help='Start link')
    parser.add_argument('end', type=str, default=None, help='End link')
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), nargs='?', default=None, help='File to load. Use - for stdin')
    parser.add_argument('-o', '--output', type=argparse.FileType('w'), default=None, help='Dump file to YAML')
    args = parser.parse_args()

    if args.file is None:
        print('Reading urdf from parameter server')
        rospy.init_node('parser', anonymous=True)
        if rospy.get_param('/robot_description', False):
            robot = URDF.from_parameter_server()
        else:
            print('Urdf not set on parameter server')
            return
    else:
        print('Reading urdf from file')
        robot = URDF.from_xml_string(args.file.read())

    if (args.start is None) or (args.end is None):
        print('start or end link not set')
        return
    else:
        yaml_dict = get_scope_yaml(robot, args.start, args.end)

    if args.output is None:
        print(yaml.dump(yaml_dict, default_flow_style=False))
    else:
        print('Writing yaml to ' + args.output.name)
        args.output.write( yaml.dump(yaml_dict, default_flow_style=False) )


if __name__ == '__main__': main()
