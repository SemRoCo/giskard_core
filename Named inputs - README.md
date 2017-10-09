Short overview of the implementation of the named inputs

There are five types of named inputs:
  - Joints: Single scalar values that can be changed by controllable constraints
  - Scalars: Single scalar values that are not controllable
  - Vectors: 3d vectors consisting of three scalars "x,y,z"
  - Rotations: 3d rotations defined as axis-angle rotations. They are broken up into four scalars: "x,y,z,a"
  - Frames: 3d frames consisting of an orientation, in axis-angle notation, and a translation. They are defined by seven scalars: "rx,ry,rz,a,x,y,z".

These inputs can be declared in YAML using the keywords "input-joint", "input-scalar", "input-vec3", "input-rotation" and "input-frame", followed by a string which is interpreted as their name during the controller generation.

The C++ side of the System

The datatypes of the inputs are encoded in the enum "giskard_core::InputType".

Much like the rest of the controller components, there are two types of inputs.

The first type is an intermediary serving as conduit between parsing and controller generation. This intermediary is defined in "specifications.hpp" and has rules for YAML conversion. All of the intermediary input types are subclasses of their respective mathematical type (e.g. DoubleSpec, VectorSpec...) and the "InputSpec"-class. This class stores the input's type and name and implements a few methods for easy equality checking.

The second input type is used during the runtime of the controller. Due to the way the controller generation is structured, the inputs are managed and assigned by the "giskard_core::Scope"-class. The inputs are declared within that class as public inner classes. (A debatable design choice) 
The runtime inputs share the common superclass "giskard_core::Scope::AInput". This abstract class stores the index at which this input begins in the observable's vector, as well as the input's name. The purely virtual method "get_type()" is implemented by the subclasses to avoid mismatching the type of an input due to faulty construction. The subclasses also store the KDL-expressions which are used during the runtime of the controller. All input classes also have a static method providing a the name of the class as a string. This is used by a few templates to generate informative error messages.

The "giskard_core::Scope"-class has been extended with functionality to add and find inputs. The methods follow the simple naming convention "add_[TYPE]_input(...)" and expect a single parameter specifying the name of the new input. There are two possible errors during input creation:
  1. An input with the same name but different type already exists.
  2. A joint is added after an input of which is not a joint. This is because the controllables are expected to be first in the observable's vector. (And it actually does not prevent controlled joints being created after non-controlled ones)
If an error occurs during the creation of an input, an exception is thrown and the input set is not changed.
To find specific inputs, the methods "has_input(...)" and "find_input(...)" can be used. The first returns "true" if the input exists and "false" otherwise. The second returns a shared pointer to the input if it exists and throws an exception otherwise. For both methods exists a templated overload which can be used to find an input of a specific type. "has_input<Scope::ScalarInput>(...)" for example will only return "true" if the input exists and is a scalar. 

Additionally methods have been added to examine all inputs at once:
  - "get_input_names()" returns the names of all inputs
  - "get_inputs()" returns a vector of all inputs. A templated overload allows to specify a type.
  - "get_input_map()" returns a map of all inputs, ordered by their names and associated with them. A templated overload allows to specify a type.

The "giskard_core::QPController"-class has been extended by functionality for setting input's values in an observable vector. All of these methods are named "set_input(...)" and require an observable's vector, a name and the values to set as inputs. The method is overloaded to make setting inputs as easy as possible. For every datatype exists a simple overload accepting the data as a sequence of doubles. On top that exist overloads for a few Eigen and KDL datatypes. It should be noted that none of these methods do any kind of validity checking. Possible errors that can occur during the setting of an input:
  1. No input with the given name exists
  2. The input exists but is of a different type
  3. The input exists and matches the type, but the observable's vector is too small
In all these cases an exception is raised and the observable's vector is remains unchanged.

Aside from the methods for setting inputs, a few others have been added to minimize the need to interact with the underlying scope. "get_input_names()", "get_inputs()", "get_input_map()" can be accessed directly using the controller object.
Further the method "get_command_map()" has been added to give a direct and handy association of (joint-)names and commands.

Lastly let's talk about the generation step.

Both generation methods for controllers and scopes in "expression_generation.hpp" have been changed. The generation of the controller now assembles the list of controllables beforehand and passes the to the method generating the scope. The scope generation starts off by collecting recursively collecting all InputSpecs in the "ScopeSpec". It then iterates over the list of controllables and adds the respective joints to the new scope. If there is no matching joint for a controllable, an exception is raised. After adding all controlled joints, the generator iterates  over all inputs for a second time and adds the remaining inputs to the scope. This is followed by the standard scope generation process.
The reordering process is neccessary to ensure that all controllable joints are at the front of the observable's vector. 

The controllable constraints ("specifications.hpp") have been changed to relate to the names of joint inputs instead of inputs in the observable's vector.

All unit tests have been updated to work with the new system and test the new functionality. A few now have ToDos in them because it was not clear what they test and how this changes due to the overhaul. The recursive finding of inputs has not been tested for all spec-classes.

A possible idea for the future: Move the set-functionality to the input classes to minimize the interactions with the controller. Not sure yet how much of a gain this is. 
