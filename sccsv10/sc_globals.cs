namespace _sc_core_systems
{
    public interface sc_globals
    {
        _sc_core_systems.sc_console.sc_console_core SC_CONSOLE_CORE { get; set; }
        _sc_core_systems.sc_console.sc_console_writer SC_CONSOLE_WRITER { get; set; }
        _sc_core_systems.sc_console.sc_console_reader SC_CONSOLE_READER { get; set; }
        _sc_core_systems.sc_core.sc_globals_accessor SC_GLOBALS_ACCESSORS { get; set; }
        int _Activate_Desktop_Image { get; set; }

    }
}
