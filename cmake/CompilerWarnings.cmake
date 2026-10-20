function(hik_quad_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(HIK_QUAD_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
    return()
  endif()

  set(_warnings
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wcast-qual
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wformat=2
    -Wdouble-promotion
    -Wnull-dereference
  )
  target_compile_options(${target} PRIVATE ${_warnings})
  if(HIK_QUAD_WARNINGS_AS_ERRORS)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
endfunction()
