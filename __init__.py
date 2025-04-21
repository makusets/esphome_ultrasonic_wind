import esphome.codegen as cg #always required
import esphome.config_validation as cv  #always required




#keys that we fill in this file, capitalised long names for keys, use underscores
#this keys are not imported from the yaml file

CONF_MY_REQUIRED_KEY = 'my_required_key' 
CONF_MY_OPTIONAL_KEY = 'my_optional_key'  


#all keys from above should be included in the config schema, here we determine whether they are
#optional or not and set, asign data type and default values, best practice to set a default value if possible
#

CONFIG_SCHEMA = cv.Schema({    
  cv.Required(CONF_MY_REQUIRED_KEY): cv.string,
  cv.Optional(CONF_MY_OPTIONAL_KEY, default=10): cv.int_,
}).extend(cv.COMPONENT_SCHEMA)


import esphome.codegen as cg  #standard syntax, always required, this is the code generation section

async def to_code(config):  #standard syntax
    var = cg.new_Pvariable(config[CONF_ID])   #standard syntax
    await cg.register_component(var, config)   #standard syntax

    cg.add(var.set_my_required_key(config[CONF_MY_REQUIRED_KEY]))
