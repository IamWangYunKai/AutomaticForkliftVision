package.path = package.path .. ";./lua_scripts/?.lua"
package.path = package.path .. ";./lua_scripts/utils/?.lua"
package.path = package.path .. ";./lua_scripts/worldmodel/?.lua"

require("utils")
require("task")
