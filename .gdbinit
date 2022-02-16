set confirm off
set print static-members off
set print pretty on
set print union off
catch throw
set $<dummy_var> = 3
#break /usr/local/include/boost/test/impl/test_tools.ipp:351
#break boost::test_tools::tt_detail::report_assertion
