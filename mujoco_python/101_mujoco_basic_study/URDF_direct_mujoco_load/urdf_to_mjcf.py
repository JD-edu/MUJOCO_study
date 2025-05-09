from robosuite.models.robots.robot_model import URDFConverter

converter = URDFConverter("jdcobot_100_description.urdf")
mjcf_model = converter.mjcf_model
mjcf_model.write_xml("jdcobot_100_description.xml")
