#[macro_export]
macro_rules! ignore_nonfatal {
    ($code:expr) => {
        let result: ::modular_flow::graph::Result<()> = do catch {
            $code
            Ok(())
        };
        match result {
            Err(Error::Aborted) => result?,
            Err(_) => {},
            Ok(_) => {},
        }
    }
}

